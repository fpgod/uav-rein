from socket import *
import threading
import time
import numpy as np
import pandas as pd

connected_port = []


UAV_Red_Att = pd.read_table('UAV_Red_Att.txt', sep='\t',engine='python')
UAV_Red_pos = pd.read_table('UAV_Red_pos.txt', sep='\t',engine='python')
UAV_Blue_Att = pd.read_table('UAV_Blue_Att.txt', sep='\t',engine='python')
UAV_Blue_pos = pd.read_table('UAV_Blue_pos.txt', sep='\t',engine='python')
data_red = pd.concat([UAV_Red_pos, UAV_Red_Att], axis=1, join='outer')
data_blue = pd.concat([UAV_Blue_pos, UAV_Blue_Att], axis=1, join='outer')
np_data_red = np.array(data_red)
np_data_blue = np.array(data_blue)
tmp = np.array(['-69.651 284.069 42.000 ', '0.000 0.000 76.995 '])
np_data_red = np.vstack((tmp, np_data_red))
tmp = np.array(['409.441 19.997 46.000 ', '0.000 0.000 -181.368 '])
np_data_blue = np.vstack((tmp, np_data_blue))


class Sendmsg:
    def __init__(self, socket_server):
        self.buffsize = 2048
        self.s = socket_server
        self.conn_list = []
        self.conn_dt = {}
        self.drone_num = 50
        self.recs()
        # self.t1 = threading.Thread(target=self.recs, args=(), name='rec')
        self.t2 = threading.Thread(target=self.sds, args=(), name='send')

    def tcplink(self, sock, addr):
        while True:
            try:
                recvdata = sock.recv(self.buffsize).decode('utf-8')
                print(recvdata, addr)
                if not recvdata:
                    break
            except:
                sock.close()
                print(addr, 'offline')
                _index = self.conn_list.index(addr)
                # gui.listBox.delete(_index)
                self.conn_dt.pop(addr)
                self.conn_list.pop(_index)
                break

    def recs(self):
        # while True:
        while len(self.conn_list) < self.drone_num:
            clientsock, clientaddress = self.s.accept()
            if clientaddress[1] not in connected_port:
                self.conn_list.append(clientaddress)
                self.conn_dt[clientaddress] = clientsock
                connected_port.append(clientaddress[1])
                print('connect from:', clientaddress)

    def sds(self):
        k = 0
        time.sleep(10)
        while True:
            for i in range(len(self.conn_list)):
                if i < 40:
                    pos = np_data_red[i*100001+k][0].split(" ")
                    rpy = np_data_red[i*100001+k][1].split(" ")
                else:
                    pos = np_data_blue[(i-40) * 100001 + k][0].split(" ")
                    rpy = np_data_blue[(i-40) * 100001 + k][1].split(" ")
                msg = '_' + str(float(pos[0])*100) + '_' + str(-float(pos[1])*100) \
                      + '_' + str(float(pos[2])*100) + '_' + \
                      rpy[0] + '_' + rpy[1] + '_' + str(float(rpy[2])+180)
                # print(msg)
                self.conn_dt[self.conn_list[i]].sendall(msg.encode('utf-8'))
                # i += 1
                # if i % 40 == 0:
                #     i = 0
            time.sleep(0.001)
            k += 1

    def run(self):
        self.t2.start()


if __name__ == '__main__':
    s = socket(AF_INET, SOCK_STREAM)
    s.bind(('127.0.0.1', 9000))
    s.listen(200)  # 最大连接数
    c1 = Sendmsg(s)
    time.sleep(2)
    c1.run()
