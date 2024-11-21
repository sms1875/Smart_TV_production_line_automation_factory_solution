import socket

def main():
    server_ip = '192.168.110.107'  # DetectPanelNode가 실행되는 서버의 IP 주소
    server_port = 12345

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.connect((server_ip, server_port))
            print(f'Connected to server {server_ip}:{server_port}')

            while True:
                data = sock.recv(1024)
                if not data:
                    break
                print(f'Received: {data.decode("utf-8").strip()}')

        except ConnectionRefusedError:
            print(f'Unable to connect to {server_ip}:{server_port}')

if __name__ == '__main__':
    main()