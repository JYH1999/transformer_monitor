import socket
import threading


class TcpClient:

    def __init__(self):
        self.ip = "8.136.87.151"
        self.port = 4950
        self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_client.connect((self.ip, self.port))

    def getMessage(self):
        while True:
            data, addr = self.tcp_client.recvfrom(1024)
            print(f'\nServer> {data.decode()}\nClient> ')

    def sendMessage(self, message):
        self.tcp_client.send(message.encode(encoding='utf-8'))

    def sendMessages(self):
        flag = True
        while flag:
            message = input("Client> \n")
            self.sendMessage(message)
            if message == 'exit':
                self.closeClient()
                flag = False

    def closeClient(self):
        self.tcp_client.close()
        print("客户端已关闭")


if __name__ == '__main__':
    T = TcpClient()
    thread_getMessage = threading.Thread(target=T.getMessage)
    thread_getMessage.setDaemon(True)
    thread_getMessage.start()
    T.sendMessages()

