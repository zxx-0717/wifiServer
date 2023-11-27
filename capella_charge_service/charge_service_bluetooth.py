import rclpy
from rclpy.node import Node
import time
import os
import threading
import crcmod.predefined
from capella_ros_msg.srv import ChargePileWifi
from capella_ros_service_interfaces.msg import ChargeState
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from capella_ros_dock_msgs.action import Dock
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy,ReliabilityPolicy,QoSProfile,HistoryPolicy
# 蓝牙模块相关的库
import asyncio
from bleak import BleakScanner,BleakClient
from bleak.exc import BleakError


# 需要实现的功能:
# 1. 实时发布topic(间隔一秒或更短):(序列号,接触状态,充电状态,对接执行状态)
# 2. 接收开始/停止充电的指令(ros服务,这里是服务端)
# 3. 接收开始/停止对接充电桩的指令(ros服务,这里是服务端)

# 自动充电流程:
# 1. agent接收到下发的充电桩坐标前往对接位置
# 2. 到达对接位置后,agent向ros发送对接充电桩的指令,此时状态为("", false, false, true)
# 3. 机器人原地旋转直到搜索到红外信号,此时状态为不变
# 4. ros连接上充电桩wifi,此时状态为("123456", false, false, true)
# 4. 机器人开始行驶,直到接触充电桩后停止,此时状态为("123456", true, false, false)
# 5. agent发送开始充电的指令,此时状态为("123456", true, true, false)

# 手动充电流程:
# 1. 用户把机器人推上充电桩
# 2. 机器人检测到红外信号,此时状态为("", false, false, false)
# 3. ros连接上充电桩wifi,此时状态为("123456", false, false, false)
# 4. 机器人发送接触状态,此时状态为("123456", true, false, false)
# 5. agent发送开始充电的指令,此时状态为("123456", true, true, false)
# 6. agent向服务器发送占用充电桩的通知

# 停止充电流程:
# 1. agent接收到下发的停止充电命令,向ros发送停止充电或对接的指令,此时状态为("123456", true, false, false)
# 2. 机器人驶离充电桩,因为不再与充电桩接触,因此认为离开充电桩,将序列号清空,此时状态为("", false, false, false)

# 话题:
# /charger/state:序列号,是否在充电,是否在对接,是否有接触等字段

# 服务:
# /charger/start: 开始充电
# /charger/stop:停止充电
# /charger/start_docking:开始对接
# /charger/stop_docking:停止对接

class BluetoothChargeServer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Bluetooth charge Server starting")
        # 是否断开与充电桩的蓝牙连接
        self.bluetooth_connected = False
        # 连接充电桩蓝牙的Mac地址
        self.charger_macid = ''
        # 蓝牙数据notify的uuid
        self.uuid_notify = None
        # 蓝牙数据write的uuid
        self.uuid_write = None
        # 初始化发送的数据
        self.send_data = None
        # 是否断开蓝牙的属性
        self.disconnect_bluetooth = False
        # 通过bssid链接充电桩WIFI服务
        self.bluetooth_concact_server = self.create_service(ChargePileWifi, 'bluetooth_bssid', self.connect_bluetooth)
        # 话题和订阅器的qos
        charger_state_qos = QoSProfile(depth=1)
        charger_state_qos.reliability = ReliabilityPolicy.RELIABLE
        charger_state_qos.history = HistoryPolicy.KEEP_LAST
        charger_state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        # 接收机器人充电时需要连接的充电桩蓝牙Mac地址的订阅器
        self.charger_id_sub = self.create_subscription(String, '/charger/id', self.charger_id_sub_callback,charger_state_qos)
        # 在机器人充电时发布刹车命令的发布器
        self.brake_publisher = self.create_publisher(Twist,'/cmd_vel', 10)
        # 定时检查WIFI是否断开
        self.check_bluetooth = self.create_timer(5, self.check_bluetooth_callback)
        # 创建充电状态发布器
        # charger_state_qos = QoSProfile(depth=1)
        # charger_state_qos.reliability = ReliabilityPolicy.RELIABLE
        # charger_state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        # self.charge_state_publisher = self.create_publisher(ChargeState, '/charger/state', charger_state_qos)
        self.charge_state_publisher = self.create_publisher(ChargeState, '/charger/state',charger_state_qos)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.charge_state_callback)
        # 初始化充电状态信息
        self.charge_state = ChargeState()
        self.charge_state.pid = ""
        self.charge_state.has_contact = False
        self.charge_state.is_charging = False
        self.charge_state.is_docking = False
        # 创建充电服务
        self.start_charge_server = self.create_service(Empty, '/charger/start', self.start_charge_callback)
        self.stop_charge_server = self.create_service(Empty, '/charger/stop', self.stop_charge_callback)
        # 接受充电桩的数据帧
        self.udp_data = []
        # 创建对接充电桩服务
        # self.start_docking_charging_pile_server = self.create_service(Empty, '/charger/start_docking',
        #                                                         self.start_docking_charging_pile_callback)
        # self.stop_docking_charging_pile_server = self.create_service(Empty, '/charger/stop_docking',
        #                                                         self.stop_docking_charging_pile_callback)
        
        self.start_docking_charging_pile_server = self.create_service(Empty, '/charger/start_docking',
                                                                self.start_docking_charging_pile_visual_callback)
        self.stop_docking_charging_pile_server = self.create_service(Empty, '/charger/stop_docking',
                                                                self.stop_docking_charging_pile_visual_callback)


    # 定时发布充电状态
    def charge_state_callback(self, ):
        self.charge_state_publisher.publish(self.charge_state)
        # 如果机器人正在充电就发布刹车
        if self.charge_state.pid != '' and self.charge_state.has_contact == True and self.charge_state.is_charging == True and self.charge_state.is_docking == False:
            brake_msg = Twist()
            brake_msg.linear.x = 0.0
            brake_msg.linear.y = 0.0
            brake_msg.linear.z = 0.0
            self.brake_publisher.publish(brake_msg)

    # 接收app发布的充电桩蓝牙Mac地址的回调函数
    def charger_id_sub_callback(self,msg):
        self.charger_macid = msg.data

    # 开始充电服务回调函数,向充电桩发送开始充电数据帧
    def start_charge_callback(self, request, response):
        # 判断是否已经连接上充电桩的wifi,没连上无法通过udp通讯
        time.sleep(0.5)
        self.get_logger().info('收到开始充电命令')
        if self.charge_state.pid != '':
            # 判断是否还没接触上充电桩，没接触上直接返回失败
            if self.charge_state.has_contact == False:
                self.get_logger().info("还未与充电桩接触,请接触好在充电。")
                return response
            # 判断是否早就已经充着电
            elif self.charge_state.is_charging == True:
                self.get_logger().info("早已经在充电了。")
                return response
            # 发送充电数据帧
            send_d = self.udp_data[:12]
            # 设置数据帧的命令码
            send_d[8] = '80'
            send_d[9] = '00'
            # 设置数据帧的长度域
            send_d[10] = '02'
            send_d[11] = '00'
            # 设置数据帧的数据域
            send_d.append('02')
            send_d.append('00')
            # 设置数据帧的校验码
            send_d.append(self.crc8(send_d))
            # 设置数据帧的结束符
            send_d.append('16')
            # 发送数据帧
            self.send_data = bytes.fromhex(''.join(send_d))
            # # 循环等待充电桩的响应结果
            t1 = time.time()
            while True:
                if self.charge_state.is_charging == True:
                    self.get_logger().info('成功开始充电！')
                    return response
                elif time.time() - t1 > 10:
                    self.get_logger().info('开始充电失败！')
                    return response
                    
        else:
            self.get_logger().info('未连接充电桩bluetooth,请先连接！')
            return response

    # 停止充电服务回调函数,向充电桩发送停止充电数据帧
    def stop_charge_callback(self, request, response):
         # 判断是否已经连接上充电桩的wifi,没连上无法通过udp通讯
        self.get_logger().info('收到停止充电命令')
        if self.charge_state.pid != '':
                # 判断当前WiFi连接状态
                if self.charge_state.is_charging == False:
                    self.get_logger().info('本来就没充电。')
                    return response
                send_d = self.udp_data[:12]
                # 设置数据帧的命令码
                send_d[8] = '80'
                send_d[9] = '00'
                # 设置数据帧的长度域
                send_d[10] = '02'
                send_d[11] = '00'
                # 设置数据帧的数据域
                send_d.append('01')
                send_d.append('00')
                # 设置数据帧的校验码
                send_d.append(self.crc8(send_d))
                # 设置数据帧的
                send_d.append('16')
                # 发送数据帧
                self.send_data = bytes.fromhex(''.join(send_d))
                # 等待充电桩回复
                t1 = time.time()
                # 循环等待充电桩的响应结果
                while True:
                    if self.charge_state.is_charging == False:
                        self.get_logger().info('成功关闭充电！')
                        return response
                    elif time.time() - t1 > 10:
                        self.get_logger().info('关闭充电失败！')
                        return response
                    
        else:
            self.get_logger().info('未连接充电桩WiFi,请先连接！')
            return response

    # 开始对接充电桩的指令(infrared)
    def start_docking_charging_pile_callback(self, request, response):
        self.get_logger().info("开始对接充电桩（红外定位）.")
        self.charge_state.is_docking = True
        return response
    
    def start_docking_charging_pile_visual_callback(self, request, response):
        self.get_logger().info("开始对接充电桩. (视觉定位)")
        self.charge_state.is_docking = True

        # connect wifi
        connect_bluetooth_request = ChargePileWifi.Request()
        connect_bluetooth_request.ssid = self.charger_macid
        connect_bluetooth_response = ChargePileWifi.Response()
        self.connect_bluetooth(connect_bluetooth_request,connect_bluetooth_response)

        if self.charge_state.pid != '':        
            self.get_logger().info('start docking action')
            self.dock_client = ActionClient(self, Dock, "dock")
            dock_msg = Dock.Goal()
            while not self.dock_client.wait_for_server(2):
                self.get_logger().info('Dock action server not available.')
            self.dock_client_sendgoal_future = self.dock_client.send_goal_async(dock_msg, self.dock_feedback_callback)
            self.dock_client_sendgoal_future.add_done_callback(self.dock_response_callback)
        return response

    def dock_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('feedback => sees_dock : {}'.format(feedback.sees_dock))

    def dock_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('goal rejected !')
            self.charge_state.is_docking = False
        else:
            self.get_logger().info('goal accepted.')
            self.charge_state.is_docking = True
            self._dock_get_future_result = goal_handle.get_result_async()
            self._dock_get_future_result.add_done_callback(self.dock_get_result_callback)

    def dock_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Dock result => is_docked: {}'.format(result.is_docked))
        self.charge_state.is_docking = False


    # 停止对接充电桩的指令
    def stop_docking_charging_pile_callback(self, request, response):        
        self.get_logger().info("停止对接充电桩. (红外定位)")
        self.dock_goal_handle = self.dock_client_sendgoal_future.result()
        cancel_goal_future = self.dock_goal_handle.cancel_goal_async()
        # self.dock_goal_handle.cancel_goal()
        cancel_response = cancel_goal_future.result()
        self.get_logger().info("Docking canceled! ")
        self.disconnect_bluetooth = True
        self.charge_state.pid = ''
        self.charge_state.has_contact = False
        self.charge_state.is_charging = False
        self.charge_state.is_docking = False

        return response
    
    def stop_docking_charging_pile_visual_callback(self, request, response):
        self.get_logger().info("停止对接充电桩. (视觉定位)")
        self.dock_goal_handle = self.dock_client_sendgoal_future.result()
        cancel_goal_future = self.dock_goal_handle.cancel_goal_async()
        # self.dock_goal_handle.cancel_goal()
        self.get_logger().info("Docking canceled! ")
        self.disconnect_bluetooth = True
        self.charge_state.pid = ''
        self.charge_state.has_contact = False
        self.charge_state.is_charging = False
        self.charge_state.is_docking = False

        return response

    # 连接充电桩蓝牙
    def connect_bluetooth(self,request, response):
        self.charger_macid = request.ssid
        self.get_logger().info("正在重启蓝牙...")
        os.system('sudo rfkill block bluetooth') # bluetoothctl power off
        time.sleep(1)
        os.system('sudo rfkill unblock bluetooth')# bluetoothctl power on
        time.sleep(1)
        self.bluetooth_connected = None
        # bssid = '55:3E:1E:AA:EB:65' # 手机蓝牙
        # if self.charger_macid == '':
        #     self.get_logger().info("id = '' ...")
        #     self.charger_macid = "94:C9:60:43:C0:6D" # 蓝牙充电桩-1 
        #     # bssid = "94:C9:60:43:BE:67" # 蓝牙充电桩-2
        # 连接网络
        self.get_logger().info(f"connect bluetooth {self.charger_macid}")
        self.thread_bule = threading.Thread(target=self.bluetooth_thread,kwargs={'mac_address':self.charger_macid},daemon=True)
        self.thread_bule.start()
        self.get_logger().info("蓝牙线程已开启。")
        t1 = time.time()
        while True:
            if time.time() - t1 > 10:
                self.get_logger().info("蓝牙连接超时。")
                self.bluetooth_connected = False
                break
            elif self.charge_state.pid == "" and self.bluetooth_connected == None:
                time.sleep(1)
                self.get_logger().info("蓝牙连接中...")
                continue
            else:
                break
        print('self.bluetooth_connected',self.bluetooth_connected)
        if self.bluetooth_connected == True:
            self.get_logger().info('蓝牙连接成功.')
            response.success = True
        else:
            self.get_logger().info('蓝牙连失败.')
            response.success = False
        return response

    # 创建bleak客户端
    async def create_bleakclient(self,address):
        # 搜索附近的蓝牙
        # self.get_logger().info("正在搜索附近的蓝牙......\n")
        # device_dict = {}
        # devices = await BleakScanner().discover(return_adv=True)
        # self.get_logger().info(f'搜到{len(devices)}个蓝牙信号。')
        # self.get_logger().info('MAC地址            name')
        # # print(devices)
        # for key in devices:
        #     self.get_logger().info(f'{key} => {devices[key][1].local_name}')
        # for d in devices:
        #     self.get_logger().info(f'{d.address} {d.}')
        #     device_dict[f'{d.address}'] = f'{d.name}'
        # # 判断要连接的蓝牙是否在搜索到的蓝牙里面
        # if address in device_dict.keys():
        try:
            # 开始连接蓝牙
            print("开始创建bleakclient")
            async with BleakClient(address) as self.bleak_client:
                self.disconnect_bluetooth = False
                if self.bleak_client.is_connected():
                    # 获取蓝牙的服务
                    services = self.bleak_client.services
                    for service in services:
                        print('服务的uuid：', service.uuid)
                        for character in service.characteristics:
                            # print('特征值uuid：', character.uuid)
                            # print('特征值属性：', character.properties)
                            # 获取发送数据的蓝牙服务uuid
                            if character.properties == ['write-without-response', 'write']:
                                self.uuid_write = character.uuid
                            # 获取接收数据的蓝牙服务uuid
                            elif character.properties == ['read', 'notify']:
                                self.uuid_notify = character.uuid
                            else:
                                continue
                        print('*************************************')
                else:
                    self.bluetooth_connected = False
                self.charge_state.pid = address
                self.bluetooth_connected = True
                while self.bleak_client.is_connected:
                    await self.bleak_client.start_notify(self.uuid_notify, self.notify_data)
                    if self.send_data is not None:
                        await self.bleak_client.write_gatt_char(self.uuid_write,self.send_data)
                        self.send_data = None
                    if self.disconnect_bluetooth:
                        await self.bleak_client.disconnect()
                        self.get_logger().info(f"已断开蓝牙。")
                        break
                self.charge_state.pid = ""
        except Exception as e:
            self.bluetooth_connected = False
            self.charge_state.pid = ""
            # self.get_logger().info(e)
        # else:
        #     self.get_logger().info(f"当前连接的wifi(bssid:{address})未找到。")
        #     self.bluetooth_connected= False
        #     self.charge_state.pid = ""
        
    def bluetooth_thread(self,mac_address):
        asyncio.run(self.create_bleakclient(mac_address))

    # 接收蓝牙数据的回调函数，解析充电桩发送的数据帧
    def notify_data(self,sender,data ):
        # 接受服务端的数据帧
        # self.get_logger().debug('-------------------receive data---------------------')
        # 将数据解码
        data = ','.join('{:02x}'.format(x) for x in data).replace(' ','')
        # 将数据帧转化为列表
        data_list = data.split(',')
        # self.get_logger().debug('解析后的数据为：', data_list)
        # self.get_logger().debug(f'收到服务器的信息: {data}')
        # self.get_logger().debug(f'解析后的数据为: {data_list}', )
        # self.get_logger().debug(f'数据列表长度为: {len(data_list)} 字节')
        # self.get_logger().debug(f'帧起始符(6BH,1字节): {data_list[0]}')
        # self.get_logger().debug(f'地址域(4字节): {data_list[1:5]}')
        # self.get_logger().debug(f'帧起始符(6BH,1字节): {data_list[5]}')
        # self.get_logger().debug(f'帧序号(2字节): {data_list[6:8]}')            
        # self.get_logger().debug(f'命令码(2字节): {data_list[8:10]}')
        # self.get_logger().debug(f'长度域(2字节): {data_list[10:12]}')
        # self.get_logger().debug(f'数据域: {data_list[12:-2]}')
        # self.get_logger().debug(f'校验码(1字节): {data_list[-2]}')
        # self.get_logger().debug(f'结束符(16H,1字节): {data_list[-1]}')
        # self.get_logger().debug(f"正在校验信息......")
        # 校验数据
        crc8_ = self.crc8(data_list[:-2])
        if crc8_ == data_list[-2].upper():
            # self.get_logger().debug('数据校验通过！')
            # self.get_logger().info('解析后的数据为：{}'.format(data_list))
            self.udp_data = data_list
            # 判断机器人与充电桩的接触状态与充电状态
            # 通过命令码是否是充电桩工作状态的信息帧
            if data_list[8:10] == ['00', '21']:
                if data_list[12:-2][0] == '00':
                    self.charge_state.is_charging = False
                elif data_list[12:-2][0] == '01':
                    self.charge_state.is_charging = True
                else:
                    self.get_logger().info('is_charging 数据段数据错误。')
                if data_list[12:-2][5] == '00':
                    self.charge_state.has_contact = False
                elif data_list[12:-2][5] == '01':
                    self.charge_state.has_contact = True
                    self.charge_state.is_docking = False
                    # now_time = self.get_clock().now()
                    # self.charge_state.stamp = now_time.to_msg()
                else:
                    self.get_logger().info('has_contact 数据段数据错误。')
        # else:
            # self.get_logger().debug(f'self crc: {crc8_}')
            # self.get_logger().debug(f'recv crc: {data_list[-2].upper()}')
            # self.get_logger().info('数据未通过校验,舍弃数据！')
            # self.get_logger().info('----------------------------')
        

    # 定时检查蓝牙是否断开的回调函数data
    def check_bluetooth_callback(self, ):
        if self.charge_state.pid == '':
            self.charge_state.pid = ''
            self.charge_state.has_contact = False
            self.charge_state.is_charging = False
        try:
            if not self.bleak_client.is_connected:
                self.charge_state.pid = ''
                self.charge_state.has_contact = False
                self.charge_state.is_charging = False
        except:
            self.charge_state.pid = ''
            self.charge_state.has_contact = False
            self.charge_state.is_charging = False

    # CRC-8/MAXIM　x8+x5+x4+1  循环冗余校验 最后在取了反的
    # 计算校验码
    def crc8(self, data):
        crc8 = crcmod.predefined.Crc('crc-8-maxim')
        # crc8.update(bytes().fromhex(' '.join(data)))
        self.get_logger().debug(f'data: {data}')
        self.get_logger().debug(f"data_join: {' '.join(data)}")

        crc8.update(bytes().fromhex(' '.join(data)))
        crc8_value = hex(~crc8.crcValue & 0xff)[2:].upper()
        crc8_value = crc8_value if len(crc8_value) > 1 else '0' + crc8_value
        return crc8_value

    # 析构函数
    def __del__(self, ):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = BluetoothChargeServer('bluetooth_charge_server')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
