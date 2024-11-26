import websocket
import threading
import time

def on_message(ws, message):
    """서버로부터 메시지를 수신했을 때 실행"""
    print(f"Received: {message}")

def on_error(ws, error):
    """오류가 발생했을 때 실행"""
    print(f"Error: {error}")

def on_close(ws, close_status_code, close_msg):
    """WebSocket 연결이 닫혔을 때 실행"""
    print("WebSocket closed")

def on_open(ws):
    """WebSocket 연결이 열렸을 때 실행"""
    print("Connection successful")

def main():
    server_url = "ws://192.168.26.47:8000/socket/ws/robodk/"  # WebSocket URL

    # WebSocket 클라이언트 생성
    ws = websocket.WebSocketApp(
        server_url,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close,
        on_open=on_open
    )

    # WebSocket 연결 유지
    websocket_thread = threading.Thread(target=ws.run_forever, daemon=True)
    websocket_thread.start()

    try:
        while True:
            time.sleep(1)  # WebSocket 연결 유지
    except KeyboardInterrupt:
        print("Closing WebSocket...")
        ws.close()

if __name__ == '__main__':
    main()
