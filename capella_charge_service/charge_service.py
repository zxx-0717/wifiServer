import rclpy
from rclpy.node import Node
import time
import socket
import threading
import crcmod.predefined
from wpa_supplicant.core import WpaSupplicantDriver
from twisted.internet.selectreactor import SelectReactor
from capella_ros_msg.srv import ChargePileWifi
from capella_ros_service_interfaces.msg import ChargeState
from std_srvs.srv import Empty
# from irobot_create_msgs.action import Dock // for dock_visual pkg
from capella_ros_dock_msgs.action import Dock
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import QoSProfile

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

class WifiServer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Wifi Server starting")
        # 创建一个无线对象
        # 获取无线网卡
        self.reactor = SelectReactor()
        threading.Thread(target=self.reactor.run, kwargs={'installSignalHandlers': 0}, daemon=True).start()
        time.sleep(1)
        self.driver = WpaSupplicantDriver(self.reactor)
        self.supplicant = self.driver.connect()
        self.declare_parameter('wifi_interface', 'wlp2s0')
        self.wifi_interface = self.get_parameter('wifi_interface').get_parameter_value().string_value
        try:
            self.iface = self.supplicant.get_interface(self.wifi_interface)
        except:
            self.iface = self.supplicant.create_interface(self.wifi_interface)
        # WIFI链接状态
        self.wifi_c = False
        # 记录bssid
        self.bssid = ''
        # 通过bssid链接充电桩WIFI服务
        self.wifi_concact_server = self.create_service(ChargePileWifi, 'wifi_bssid', self.connect_wifi)
        # 定时检查WIFI是否断开
        self.check_wifi = self.create_timer(5, self.check_wifi_callback)
        # 创建充电状态发布器
        # charger_state_qos = QoSProfile(depth=1)
        # charger_state_qos.reliability = ReliabilityPolicy.RELIABLE
        # charger_state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        # self.charge_state_publisher = self.create_publisher(ChargeState, '/charger/state', charger_state_qos)
        self.charge_state_publisher = self.create_publisher(ChargeState, '/charger/state', 50)
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

        # 创建一个UDP连接对象
        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 固定IP及端口号
        self.client.bind(('192.168.4.5', 8080))
        # 创建一个单独线程接受充电桩的数据帧
        task1 = threading.Thread(target=self.receive_data, daemon=True)
        task1.start()

    # 定时发布充电状态
    def charge_state_callback(self, ):
        self.charge_state_publisher.publish(self.charge_state)

    # 充电服务回调函数,向充电桩发送开始和停止充电数据帧
    def start_charge_callback(self, request, response):
        # 判断是否已经连接上充电桩的wifi,没连上无法通过udp通讯
        time.sleep(0.5)
        self.get_logger().info('收到开始充电命令')
        if self.wifi_c == True:
            # 判断开始充电还是结束充电
            # 发送充电数据帧
            if self.charge_state.has_contact == False:
                self.get_logger().info("还未与充电桩接触,请接触好在充电。")
                return response
            elif self.charge_state.is_charging == True:
                self.get_logger().info("早已经在充电了。")
                return response
            send_d = self.udp_data[:12]
            # 设置数据帧的命令码
            send_d[8] = '80'
            send_d[9] = '00'
            # 设置数据帧的长度域
            send_d[10] = '01'
            send_d[11] = '00'
            # 设置数据帧的数据域
            send_d.append('02')
            # 设置数据帧的校验码
            send_d.append(self.crc8(send_d))
            # 设置数据帧的结束符
            send_d.append('16')
            # 发送数据帧
            self.send_data(send_d)
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
            self.get_logger().info('未连接充电桩WiFi,请先连接！')
            return response

    def stop_charge_callback(self, request, response):
         # 判断是否已经连接上充电桩的wifi,没连上无法通过udp通讯
        self.get_logger().info('收到停止充电命令')
        if self.wifi_c == True:
                # 判断当前WiFi连接状态
                if self.charge_state.is_charging == False:
                    self.get_logger().info('本来就没充电。')
                    return response
                send_d = self.udp_data[:12]
                self.get_logger().info('try to stop charging')
                # 设置数据帧的命令码
                send_d[8] = '80'
                send_d[9] = '00'
                # 设置数据帧的长度域
                send_d[10] = '01'
                send_d[11] = '00'
                # 设置数据帧的数据域
                send_d.append('01')
                # 设置数据帧的校验码
                send_d.append(self.crc8(send_d))
                # 设置数据帧的
                send_d.append('16')
                # 发送数据帧
                self.send_data(send_d)
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
        self.wifi_connect_cli = self.create_client(ChargePileWifi, '/wifi_bssid')
        wifi_msg = ChargePileWifi.Request()
        wifi_msg.ssid = 1
        self.wifi_connect_future = self.wifi_connect_cli.call_async(wifi_msg)


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
        self.charge_state.is_docking = False

        return response
    
    def stop_docking_charging_pile_visual_callback(self, request, response):        
        self.get_logger().info("停止对接充电桩. (视觉定位)")
        self.dock_goal_handle = self.dock_client_sendgoal_future.result()
        cancel_goal_future = self.dock_goal_handle.cancel_goal_async()
        # self.dock_goal_handle.cancel_goal()
        self.get_logger().info("Docking canceled! ")
        self.charge_state.is_docking = False

        return response

    # 通过bssid连接网络
    def connect_wifi(self, request, response, password=None):
        if request.ssid == 0:
            response.success = False
            return response
        bssid = hex(request.ssid)[2:]
        bssid = [bssid[x:x+2] for x in range(0,len(bssid),2)]
        bssid = reversed(bssid)
        bssid =":".join(bssid).upper()
        self.get_logger().info('try to connect wifi: {}'.format(bssid))
        bssid = 'CA:C9:A3:98:E1:65' # charge bssid  CA:C9:A3:98:E1:65, bssid_u64: 112019602917834 
        # bssid = 'C8:C2:FA:23:B2:34' # zxx 
        password = 'cqtj8888'
        net = None
        # 扫描当前环境中的WiFi网络
        scan_results = self.iface.scan(block=True)

        for bss in scan_results:
            self.get_logger().debug(f'wifi: {bss.get_ssid()}')
            self.get_logger().debug(f'bssid: {bss.get_bssid()}')
            self.get_logger().debug('-----------------------------')

        # 在扫描结果中循环找到要链接的WiFi
        for bss in scan_results:
            if bss.get_bssid() == bssid:  # 'CA:C9:A3:98:E1:65'
                network_cfg = {}
                network_cfg['ssid'] = bss.get_ssid()
                # network_cfg['bssid'] = bss.get_bssid()
                network_cfg['bssid'] = bssid
                if password is not None:
                    network_cfg['psk'] = password
                    network_cfg['key_mgmt'] = 'WPA-PSK'
                else:
                    network_cfg['key_mgmt'] = 'NONE'
                net = network_cfg
                break
        if net == None:
            self.get_logger().info(f'想要连接的wifi(bssid:{bssid})不存在。')
            response.success = False
            return response
        # 添加网络配置信息
        net = self.iface.add_network(net)
        # 连接网络
        self.iface.select_network(net.get_path())
        # 等待连接成功或者连接超时,10秒
        start_time = time.time()
        while True:
            if time.time() - start_time <= 10 and self.iface.get_state() != 'completed':
                self.get_logger().info(self.iface.get_state())
                continue
            elif time.time() - start_time > 10:
                self.get_logger().info('连接超时！')
                response.success = False
                return response
            elif self.iface.get_state() == 'completed':
                break
        current_bssid = self.iface.get_current_bss().get_bssid()
        if current_bssid == bssid:
            self.get_logger().info(f"连接wifi(bssid:{bssid})成功。")
            # 创建一个单独线程接受充电桩的数据帧
            # task1 = threading.Thread(target=self.receive_data, daemon=True)
            # task1.start()
            self.wifi_c = True
            response.success = True
            self.bssid = bssid
            self.charge_state.pid = bssid
            return response
        else:
            self.get_logger().info(f"当前连接的wifi(bssid:{current_bssid})与想要连接的wifi(bssid:{bssid})不匹配。")
            response.success = False
            return response

    # 通过UDP链接向充电桩发送数据帧
    def send_data(self, data):
        self.get_logger().info('*******************send data via udp**********************')
        data = bytes.fromhex(''.join(data))
        try:
            if data == 'quit':
                self.get_logger().info('结束发送数据进程。')

            self.client.sendto(data, ('192.168.4.1', 7000))
        except:
            self.get_logger().info("信息发送失败,请检查服务。")
            self.client.close()

    # 通过UDP链接接收充电桩发送的数据帧
    def receive_data(self, ):
        while True:
            # 接受服务端的数据帧
            self.get_logger().debug('-------------------receive data---------------------')
            data, addr = self.client.recvfrom(1024)
            self.get_logger().debug(f'data: {data}')
            self.get_logger().debug(f'addr: {addr}')
            # 将数据解码
            if data == 'quit':
                self.get_logger().info('收到服务器的结束命令。')
                break
            # 将数据帧转化为列表
            data_list = [hex(x)[2:] if len(hex(x)[2:]) >1 else "0"+hex(x)[2:] for x in data]
            self.get_logger().debug(f'收到服务器的信息: {data}')
            self.get_logger().debug(f'解析后的数据为: {data_list}', )
            self.get_logger().debug(f'数据列表长度为: {len(data_list)} 字节')
            self.get_logger().debug(f'帧起始符(6BH,1字节): {data_list[0]}')
            self.get_logger().debug(f'地址域(4字节): {data_list[1:5]}')
            self.get_logger().debug(f'帧起始符(6BH,1字节): {data_list[5]}')
            self.get_logger().debug(f'帧序号(2字节): {data_list[6:8]}')            
            self.get_logger().debug(f'命令码(2字节): {data_list[8:10]}')
            self.get_logger().debug(f'长度域(2字节): {data_list[10:12]}')
            self.get_logger().debug(f'数据域: {data_list[12:-2]}')
            self.get_logger().debug(f'校验码(1字节): {data_list[-2]}')
            self.get_logger().debug(f'结束符(16H,1字节): {data_list[-1]}')
            self.get_logger().debug(f"正在校验信息......")
            # 校验数据
            crc8_ = self.crc8(data_list[:-2])
            self.get_logger().debug(f'self crc: {crc8_}')
            self.get_logger().debug(f'recv crc: { data_list[-2].upper()}')
            self.get_logger().debug(f'----------------------------')
            if crc8_ == data_list[-2].upper():
                self.get_logger().debug('数据校验通过！')
                self.udp_data = data_list
                # 判断机器人与充电桩的接触状态与充电状态
                # 通过命令码是否是充电桩工作状态的信息帧
                if data_list[8:10] == ['00', '21']:
                    if data_list[12:-2][0] == '00':
                        self.charge_state.is_charging = False
                    elif data_list[12:-2][0] == '01':
                        self.charge_state.is_charging = True
                    else:
                        self.get_logger().debug('is_charging 数据段数据错误。')
                    if data_list[12:-2][-1] == '00':
                        self.charge_state.has_contact = False
                    elif data_list[12:-2][-1] == '01':
                        self.charge_state.has_contact = True
                        self.charge_state.is_docking = False
                        # now_time = self.get_clock().now()
                        # self.charge_state.stamp = now_time.to_msg()
                    else:
                        self.get_logger().debug('has_contact 数据段数据错误。')
            else:
                self.get_logger().debug(f'self crc: {crc8_}')
                self.get_logger().debug(f'recv crc: {data_list[-2].upper()}')
                self.get_logger().info('数据未通过校验,舍弃数据！')
                self.get_logger().info('----------------------------')
        self.get_logger().info(f'接收数据线程已结束')

    # 定时检查WIFI是否断开的回调函数data
    def check_wifi_callback(self, ):
        if self.iface.get_current_bss() is not None:
            if self.iface.get_current_bss().get_bssid() == "CA:C9:A3:98:E1:65": # need to fix with a topic for request bssid
                pass
            else:
                self.get_logger().info("current_bss: {}".format(self.iface.get_current_bss()))
                self.get_logger().info("current_bssid: {}".format(self.iface.get_current_bss().get_bssid()))
                self.get_logger().info("self.bssid: {}".format(self.bssid))
                self.get_logger().info(f"wifi state: {self.iface.get_state()}")
                self.wifi_c = False
                self.charge_state.pid = ''
                self.charge_state.has_contact = False
                self.charge_state.is_charging = False
                self.charge_state.is_docking = False
                self.bssid = ''

        else:
            self.get_logger().info("wifi not connected")
            self.get_logger().info(f"wifi state: {self.iface.get_state()}")
            self.wifi_c = False
            self.charge_state.pid = ''
            self.charge_state.has_contact = False
            self.charge_state.is_charging = False
            self.charge_state.is_docking = False
            self.bssid = ''

    # CRC-8/MAXIM　x8+x5+x4+1  循环冗余校验
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
        self.reactor.stop()


def main(args=None):
    rclpy.init(args=args)
    node = WifiServer('wifi_server')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
