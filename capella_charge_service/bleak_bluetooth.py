import asyncio
import time
from bleak import discover,BleakScanner,BleakClient
import threading
import crcmod.predefined

class BlueTooth_(object):
    def __init__(self,address):
        self.send_data = None
        t = threading.Thread(target=self.test1)
        t.start()
        print('线程已开启')
        time.sleep(10)
        self.send_data = bytes.fromhex(''.join(['6b','00','00','00','00','6b','00','00','80','00','02','00','02','00','8F','16']))
        time.sleep(10)
        self.send_data = bytes.fromhex(''.join(['6b','00','00','00','00','6b','00','00','80','00','02','00','01','00','DA','16']))

    # 创建bleak客户端
    async def create_bleakclient(self,address):
        async with BleakClient(address) as self.bleak_client:
            self.bleak_client.pair()
            while self.bleak_client.is_connected:
                await self.bleak_client.start_notify('49535343-1e4d-4bd9-ba61-23c647249616', self.notify_data)

                if self.send_data != None:
                    await self.bleak_client.write_gatt_char("49535343-8841-43f4-a8d4-ecbe34729bb3",self.send_data)
                    self.send_data = None
                # await asyncio.sleep(5.0)
                # # await self.bleak_client.stop_notify('49535343-1e4d-4bd9-ba61-23c647249616')

    def test1(self):
        print('thread starting')
        # 创建bleak客户端
        asyncio.run(self.create_bleakclient("94:C9:60:43:C3:DF"))

    async def test(self):
        while self.bleak_client.is_connected:
            await self.bleak_client.start_notify('49535343-1e4d-4bd9-ba61-23c647249616', self.notify_data)
        # while self.bleak_client.is_connected:
        #     print('notify_uuid: ', self.notify_uuid)
            # await self.bleak_client.start_notify(self.notify_uuid, self.notify_data)
    # **********************************************************************************************************************
    # 接收蓝牙数据的回调函数，解析充电桩发送的数据帧
    def notify_data(self,sender, data):
        # 接受服务端的数据帧
        # 将数据解码
        data = ','.join('{:02x}'.format(x) for x in data).replace(' ', '')
        # 将数据帧转化为列表
        data_list = data.split(',')
        # print('解析后的数据为：', data_list)
        # print(f'收到服务器的信息: {data}')
        # print(f'解析后的数据为: {data_list}', )
        # print(f'数据列表长度为: {len(data_list)} 字节')
        # print(f'帧起始符(6BH,1字节): {data_list[0]}')
        # print(f'地址域(4字节): {data_list[1:5]}')
        # print(f'帧起始符(6BH,1字节): {data_list[5]}')
        # print(f'帧序号(2字节): {data_list[6:8]}')
        # print(f'命令码(2字节): {data_list[8:10]}')
        # print(f'长度域(2字节): {data_list[10:12]}')
        # print(f'数据域: {data_list[12:-2]}')
        # print(f'校验码(1字节): {data_list[-2]}')
        # print(f'结束符(16H,1字节): {data_list[-1]}')
        # print(f"正在校验信息......")
        # 校验数据
        crc8_ = self.crc8(data_list[:-2])
        # print(f'self crc: {crc8_}')
        # print(f'recv crc: {data_list[-2].upper()}')
        # print(f'----------------------------')
        if crc8_ == data_list[-2].upper():
            print('数据校验通过！')
            udp_data = data_list
            # 判断机器人与充电桩的接触状态与充电状态
            # 通过命令码是否是充电桩工作状态的信息帧
            if data_list[8:10] == ['00', '21']:
                if data_list[12:-2][0] == '00':
                    print('charge_state.is_charging',False)
                elif data_list[12:-2][0] == '01':
                    print('charge_state.is_charging',True)
                else:
                    print('is_charging 数据段数据错误。')
                if data_list[12:-2][5] == '00':
                    print('charge_state.has_contact',False)
                elif data_list[12:-2][5] == '01':
                    print('charge_state.has_contact',True)
                    # print('charge_state.is_docking', False)
                    # now_time = self.get_clock().now()
                    # self.charge_state.stamp = now_time.to_msg()
                else:
                    print('has_contact 数据段数据错误。')
            if data_list[8:10] == ['00', '00']:
                print(data_list)
        else:
            pass
            # print(f'self crc: {crc8_}')
            # print(f'recv crc: {data_list[-2].upper()}')
            # print('数据未通过校验,舍弃数据！')
            # print('----------------------------')

    # # 异步接收充电桩蓝牙发送的数据帧
    # async def receive_data_(bleakclient,uuid):
    #     paired = await bleakclient.pair(protection_level=2)
    #     while paired:
    #         if bleakclient.is_connected():
    #             try:
    #                 await bleakclient.start_notify(uuid, notify_data)
    #             except Exception as e:
    #                 print(f'接收通知已停止：{e}')
    #         else:
    #             print(f'蓝牙未连接，接收数据线程已结束')
    #             break




    # # 接收蓝牙数据
    # def receive_data(bleakclient,uuid):
    #     asyncio.run(receive_data_(bleakclient,uuid))


    # CRC-8/MAXIM　x8+x5+x4+1  循环冗余校验 最后在取了反的
    # 计算校验码
    def crc8(self,data):
        crc8 = crcmod.predefined.Crc('crc-8-maxim')
        # crc8.update(bytes().fromhex(' '.join(data)))
        # print(f'data: {data}')
        # print(f"data_join: {' '.join(data)}")

        crc8.update(bytes().fromhex(' '.join(data)))
        crc8_value = hex(~crc8.crcValue & 0xff)[2:].upper()
        crc8_value = crc8_value if len(crc8_value) > 1 else '0' + crc8_value
        return crc8_value

    #
    # 发送数据
    # 向充电桩发送数据帧
    def send_data(self,bleak_client,data , uuid):
        if bleak_client is not None:
            asyncio.run(send_data_(bleak_client,data,uuid))
        else:
            print("蓝牙还未连接。")
    # bleak需要异步发送
    async def send_data_(bleak_client,data,uuid):
        try:
            await bleak_client.write_gatt_char(uuid,data)
        except Exception as e:
            print(f"发送数据失败：{e}")

if __name__ == "__main__":
    # 连接蓝牙
    bssid ='94:C9:60:43:C3:DF'# "94:C9:60:43:C3:DF"  94:C9:60:43:BE:6C
    bleakclient1 = BlueTooth_(bssid)
    # send_data1 = ['6b','00','00','00','00','6b','00','00','80','00','02','00','02','00','8F','16']
    # send_data1 = bytes.fromhex(''.join(['6b','00','00','00','00','6b','00','00','80','00','02','00','02','00','8F','16']))

    time.sleep(2000)



