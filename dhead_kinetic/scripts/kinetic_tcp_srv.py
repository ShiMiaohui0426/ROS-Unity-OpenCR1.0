#!/usr/bin/env python
import socket
import queue
import json
import threading


class kinect_tcp_server():

    def __init__(self, host='192.168.1.4', port=8080):
        self.host = host  # 主机IP
        self.port = port
        # 端口
        self.web = socket.socket()  # 创建TCP/IP套接字
        self.web.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.web.bind((self.host, self.port))  # 绑定端口
        self.web.listen(5)  # 设置最多连接数
        self.inputData = queue.Queue(1024)
        self.running = False
        print("srv start")

    def catch_data(self):
        while self.running:
            data = self.conn.recv(1024 * 10).decode()  # 获取客户端请求的数据
            data_json = json.loads(data)
            self.inputData.put_nowait(data_json)
            # print(data_json)  # 打印出接收到的数据

    def wait_connect(self):
        while True:
            self.conn, self.addr = self.web.accept()  # 建立客户端连接
            self.t = threading.Thread(target=self.catch_data)
            self.running = True
            self.t.start()
            break
        print("get connected with ：")
        print(self.addr)

    def close_connect(self):
        self.running = False

        self.conn.close()  # 关闭连接

    def send_pose(self, pose):
        data = {
            'position': {'x': pose[0], 'y': pose[1], 'z': pose[2]

                         },
            'orentation': {'x': pose[3], 'y': pose[4], 'z': pose[5], 'w': pose[6]}
        }
        str_json = json.dumps(data)
        byte_json = str_json.encode()
        self.conn.send(byte_json)

    def empty_input(self):
        return self.inputData.empty()

    def get_data(self):
        return self.inputData.get_nowait()


if __name__ == "__main__":
    srv = kinect_tcp_server()
    srv.wait_connect()
    i = 0
    while i < 5:
        if not srv.empty_input():
            print(srv.get_data())
            i = i + 1

    #    # 向客户端发送数据
    # conn.sendall(b'HTTP/1.1 200 OK\r\n\r\nHello World')
    srv.send_joints([1, 2, 3, 4, 5, 6, 7])
    srv.close_connect()
