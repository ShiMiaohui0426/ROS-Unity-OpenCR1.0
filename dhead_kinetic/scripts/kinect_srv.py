import socket
import queue
import json
import threading


class kinect_server():

    def __init__(self):
        self.host = '192.168.3.7'  # 主机IP
        self.port = 8080
        # 端口
        self.web = socket.socket()  # 创建TCP/IP套接字
        self.web.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.web.bind((self.host, self.port))  # 绑定端口
        self.web.listen(5)  # 设置最多连接数
        self.inputData=queue.Queue(1024)
        self.running=False
        print("srv start")

    def catch_data(self):
        while self.running:
            data = self.conn.recv(1024 * 10).decode()  # 获取客户端请求的数据
            data_json = json.loads(data)
            self.inputData.put_nowait(data_json)
            #print(data_json)  # 打印出接收到的数据
    def wait_connect(self):
        while True:
            self.conn, self.addr = self.web.accept()  # 建立客户端连接
            self.t=threading.Thread(target=self.catch_data)
            self.running = True
            self.t.start()
            break
        print("get connected with ：")
        print(self.addr)
    def close_connect(self):
        self.running=False

        self.conn.close()  # 关闭连接

    def send_joints(self,joints):
        data = {
            'position': {'x': joints[0], 'y': joints[1], 'z': joints[2]

            },
            'orentation': {'x': joints[3], 'y': joints[4], 'z': joints[5], 'w': joints[6]}
        }
        str_json = json.dumps(data)
        byte_json=str_json.encode()
        self.conn.send(byte_json)

    def empty_input(self):
        return self.inputData.empty()

    def get_data(self):
        return self.inputData.get_nowait()




if __name__ == "__main__":
    srv = kinect_server()
    srv.wait_connect()
    i=0
    while i < 5:
        if not srv.empty_input():
            print(srv.get_data())
            i = i+1

    #    # 向客户端发送数据
    #conn.sendall(b'HTTP/1.1 200 OK\r\n\r\nHello World')
    srv.send_joints([1,2,3,4,5,6,7])
    srv.close_connect()
