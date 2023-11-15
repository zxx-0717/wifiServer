import asyncio
from bleak import BleakClient,BleakScanner
import os
import time
import threading

async def main():
    os.system('sudo rfkill block bluetooth') # bluetoothctl power off
    time.sleep(2)
    os.system('sudo rfkill unblock bluetooth')# bluetoothctl power on
    time.sleep(2)
    a = await BleakScanner.discover()
    print(a)
    time.sleep(1)

# asyncio.run(main())

for i in range(10000):
        print(f"正在进行第{i}次测试")
        asyncio.run(main())
        # a = os.system(r'ros2 service call /bluetooth_bssid capella_ros_msg/srv/ChargePileWifi "{ssid: 1}"')
        time.sleep(2)

# async def create_bleakclient(address):
#         # # 搜索附近的蓝牙
#         # print("正在搜索附近的蓝牙......\n")
#         # device_dict = {}
#         # devices = await BleakScanner().discover()
#         # print(f'搜到{len(devices)}个蓝牙信号。')
#         # print('MAC地址            name')
#         # for d in devices:
#         #     print(d.address,d.name,d.rssi)
#         #     device_dict[f'{d.address}'] = f'{d.name}'
#         # # 判断要连接的蓝牙是否在搜索到的蓝牙里面
#         # if address in device_dict.keys():
#                 # 开始连接蓝牙
#                 print("开始创建bleakclient")
#                 async with BleakClient(address) as bleak_client:
#                         # 获取蓝牙的服务
#                         services = bleak_client.services
#                         for service in services:
#                                 print('服务的uuid：', service.uuid)
#                                 for character in service.characteristics:
#                                         # print('特征值uuid：', character.uuid)
#                                         # print('特征值属性：', character.properties)
#                                         # 获取发送数据的蓝牙服务uuid
#                                         if character.properties == ['write-without-response', 'write']:
#                                                 uuid_write = character.uuid
#                                         # 获取接收数据的蓝牙服务uuid
#                                         elif character.properties == ['read', 'notify']:
#                                                 uuid_notify = character.uuid
#                                         else:
#                                                 continue
#                                 print('*************************************')

#                 print("blue uuid:",uuid_notify)
                
#                 while bleak_client.is_connected:
#                         if num % 1000 == 0:
#                                 print('当前线程号：',threading.current_thread(),'self.num',self.num)
#                                 num += 1


# bssid = "94:C9:60:43:C3:DF"
# asyncio.run(create_bleakclient(bssid))


# class test_blue(object):
#     def __init__(self):
#         self.bluetooth_connected = False
#         # 蓝牙数据notify的uuid
#         self.uuid_notify = None
#         # 蓝牙数据write的uuid
#         self.uuid_write = None
#         # 初始化发送的数据
#         self.send_data = None
#         # 是否断开蓝牙的属性
#         self.disconnect_bluetooth = False
#         self.num = 0

#         # 连接充电桩蓝牙
#     def connect_bluetooth(self,bssid="94:C9:60:43:C0:6D"):
#         print("正在重启蓝牙...")
#         os.system('sudo rfkill block bluetooth') # bluetoothctl power off
#         time.sleep(2)
#         os.system('sudo rfkill unblock bluetooth')# bluetoothctl power on
#         time.sleep(2)
#         self.bluetooth_connected = None
#         # bssid = '55:3E:1E:AA:EB:65' # 手机蓝牙
#         # bssid = "94:C9:60:43:C0:6D" #充电桩
#         # 连接网络
#         self.thread_bule = threading.Thread(target=self.bluetooth_thread,kwargs={'mac_address':bssid},daemon=True)
#         self.thread_bule.start()
#         print("蓝牙线程已开启。")
#         t1 = time.time()
#         while True:
#             if time.time() - t1 > 30:
#                 print("蓝牙连接超时。")
#                 self.bluetooth_connected = False
#                 break
#             elif self.bluetooth_connected == None:
#                 time.sleep(1)
#                 print("蓝牙连接中...")
#                 continue
#             else:
#                 break
#         print('self.bluetooth_connected',self.bluetooth_connected)
#         if self.bluetooth_connected == True:
#             print('蓝牙连接成功.')
#         else:
#             print('蓝牙连失败.')

    
#     async def create_bleakclient(self,address):
#         # 搜索附近的蓝牙
#         print("正在搜索附近的蓝牙......\n")
#         device_dict = {}
#         devices = await BleakScanner().discover()
#         print(f'搜到{len(devices)}个蓝牙信号。')
#         print('MAC地址            name')
#         for d in devices:
#             print(d.address,d.name,d.rssi)
#             device_dict[f'{d.address}'] = f'{d.name}'
#         # # 判断要连接的蓝牙是否在搜索到的蓝牙里面
#         # if address in device_dict.keys():
#         try:
#             # 开始连接蓝牙
#             print("开始创建bleakclient")
#             async with BleakClient(address) as self.bleak_client:
#                 self.disconnect_bluetooth = False
#                 if self.bleak_client.is_connected():
#                     # 获取蓝牙的服务
#                     services = self.bleak_client.services
#                     for service in services:
#                         print('服务的uuid：', service.uuid)
#                         for character in service.characteristics:
#                             # print('特征值uuid：', character.uuid)
#                             # print('特征值属性：', character.properties)
#                             # 获取发送数据的蓝牙服务uuid
#                             if character.properties == ['write-without-response', 'write']:
#                                 self.uuid_write = character.uuid
#                             # 获取接收数据的蓝牙服务uuid
#                             elif character.properties == ['read', 'notify']:
#                                 self.uuid_notify = character.uuid
#                             else:
#                                 continue
#                         print('*************************************')
#                 else:
#                     self.bluetooth_connected = False
#                 print("blue uuid:",self.uuid_notify)
#                 self.bluetooth_connected = True
#                 while self.bleak_client.is_connected:
#                     if self.num % 1000 == 0:
#                         # print('当前线程号：',threading.current_thread(),'self.num',self.num)
#                         pass
#                     self.num += 1
#                     await self.bleak_client.start_notify(self.uuid_notify, self.notify_data)
#                     if self.send_data is not None:
#                         await self.bleak_client.write_gatt_char(self.uuid_write,self.send_data)
#                         self.send_data = None
#                     if self.disconnect_bluetooth:
#                         await self.bleak_client.disconnect()
#                         print(f"已断开蓝牙。")
#                         break
#         except Exception as e:
#             self.bluetooth_connected = False
#             print(e)
#         # else:
#         #     print(f"当前连接的wifi(bssid:{address})未找到。")
#         #     self.bluetooth_connected= False
        
#     def bluetooth_thread(self,mac_address):
#         asyncio.run(self.create_bleakclient(mac_address))

#     # 接收蓝牙数据的回调函数，解析充电桩发送的数据帧
#     def notify_data(self,sender,data ):
#         # 接受服务端的数据帧
#         # self.get_logger().debug('-------------------receive data---------------------')
#         # 将数据解码
#         data = ','.join('{:02x}'.format(x) for x in data).replace(' ','')
#         # 将数据帧转化为列表
#         data_list = data.split(',')
#         if self.num % 1000 ==0:
#             print('解析后的数据为：{}'.format(data_list))
        


# a = test_blue()
# for i in range(1000):
#     print(f'第{i}次循环')
#     a.connect_bluetooth()
#     print('当前总线程数：',threading.active_count())
#     time.sleep(2)
