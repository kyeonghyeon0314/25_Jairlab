#!/usr/bin/env python3

import rospy
import http.server
import socketserver
import threading
import webbrowser
import os
import json
import asyncio
import websockets
import time
from std_msgs.msg import String

# 📌 HTTP & WebSocket 설정
PORT = 8000
WEBSOCKET_PORT = 8765
WAYPOINTS_WEBSOCKET_PORT = 8766
WEB_DIR = os.path.join(os.path.dirname(__file__), "../web")  # 웹 폴더 경로

# 📌 ROS 설정
ROS_NODE_NAME = "gps_server"
GPS_TOPIC = "gps_data"
WAYPOINTS_TOPIC = "waypoints"

# 최신 GPS 데이터 및 Waypoints 저장 (쓰레드 안전)
latest_gps_data = None
latest_waypoints = None
data_lock = threading.Lock()

# ---------------------------
# 📌 HTTP 서버 실행 (포트 충돌 방지 추가)
# ---------------------------
class CustomHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/favicon.ico':
            self.send_response(204)
            self.end_headers()
            return
        return super().do_GET()

def start_http_server():
    """ HTTP 서버 실행 (index.html 제공) """
    os.chdir(WEB_DIR)
    try:
        with socketserver.TCPServer(("", PORT), CustomHandler) as httpd:
            rospy.loginfo(f"🌍 HTTP 서버 실행 중: http://localhost:{PORT}")
            httpd.serve_forever()
    except OSError:
        rospy.logerr(f"❌ HTTP 서버 포트({PORT}) 이미 사용 중! 기존 프로세스를 종료하세요.")

def open_browser():
    """ 웹 브라우저 자동 실행 """
    url = f"http://localhost:{PORT}/index.html"
    rospy.loginfo(f"🌐 브라우저 열기: {url}")
    webbrowser.open(url)

# ---------------------------
# 📌 ROS 노드 설정 (GPS 데이터 수신)
# ---------------------------
def gps_callback(data):
    """ ROS에서 GPS 데이터 수신 후 저장 """
    global latest_gps_data
    with data_lock:
        latest_gps_data = json.loads(data.data)  # 문자열을 JSON으로 변환
    rospy.loginfo(f"📡 ROS GPS 데이터 수신: {latest_gps_data}")

def start_ros_node():
    """ ROS 노드 초기화 및 구독 (메인 스레드에서 실행) """
    rospy.init_node(ROS_NODE_NAME, anonymous=True)
    rospy.Subscriber(GPS_TOPIC, String, gps_callback)
    rospy.loginfo(f"🚀 ROS 노드 '{ROS_NODE_NAME}' 실행 완료")

# ---------------------------
# 📌 WebSocket 서버 실행 (GPS 데이터 전송)
# ---------------------------
async def send_gps_data(websocket, path):
    """ WebSocket을 통해 웹 클라이언트로 GPS 데이터 전송 """
    while True:
        with data_lock:
            data_to_send = latest_gps_data
        gps_data = json.dumps(data_to_send) if data_to_send else json.dumps({"error": "센서 데이터 없음"})
        await websocket.send(gps_data)
        rospy.loginfo(f"📡 WebSocket 전송: {gps_data}")
        await asyncio.sleep(1)

async def start_websocket_server():
    """ WebSocket 서버 실행 """
    try:
        rospy.loginfo(f"🔗 WebSocket 서버 실행 중: ws://localhost:{WEBSOCKET_PORT}")
        async with websockets.serve(send_gps_data, "localhost", WEBSOCKET_PORT):
            await asyncio.Future()  # 무한 대기
    except OSError:
        rospy.logerr(f"❌ WebSocket 포트({WEBSOCKET_PORT}) 이미 사용 중! 기존 프로세스를 종료하세요.")

# ---------------------------
# 📌 WebSocket (웹 → ROS로 Waypoints 전송)
# ---------------------------
async def receive_waypoints(websocket, path):
    """ 웹에서 받은 경로 데이터를 ROS 토픽으로 전송 """
    global latest_waypoints
    pub = rospy.Publisher(WAYPOINTS_TOPIC, String, queue_size=10)

    async for message in websocket:
        try:
            waypoints = json.loads(message)  # JSON 파싱
            if not isinstance(waypoints, dict) or "waypoints" not in waypoints:
                rospy.logerr("❌ 잘못된 Waypoints 데이터 형식!")
                continue
            
            with data_lock:
                latest_waypoints = waypoints

            # 목적지 좌표 포함 확인
            destination = waypoints.get("destination", None)
            if destination:
                rospy.loginfo(f"📍 목적지 좌표 수신: {destination}")
            else:
                rospy.logwarn("⚠️ 목적지 좌표 없음")

            pub.publish(json.dumps(waypoints))
            rospy.loginfo(f"🗺️ 웹에서 받은 Waypoints & 목적지 데이터: {waypoints}")

        except Exception as e:
            rospy.logerr(f"❌ Waypoints 처리 오류: {e}")

async def start_waypoints_websocket():
    """ Waypoints WebSocket 서버 실행 """
    try:
        rospy.loginfo(f"🔗 Waypoints WebSocket 실행 중: ws://localhost:{WAYPOINTS_WEBSOCKET_PORT}")
        async with websockets.serve(receive_waypoints, "localhost", WAYPOINTS_WEBSOCKET_PORT):
            await asyncio.Future()
    except OSError:
        rospy.logerr(f"❌ Waypoints WebSocket 포트({WAYPOINTS_WEBSOCKET_PORT}) 이미 사용 중! 기존 프로세스를 종료하세요.")

# ---------------------------
# 📌 메인 실행부
# ---------------------------
if __name__ == '__main__':
    # ✅ 1️⃣ ROS 노드 실행 (메인 스레드에서 실행)
    start_ros_node()

    # ✅ 2️⃣ HTTP 서버 실행 (웹 제공)
    threading.Thread(target=start_http_server, daemon=True).start()
    
    time.sleep(2)  # ❗ `time` 모듈 사용 가능하도록 수정
    open_browser()  # 🌍 웹페이지 자동 실행

    # ✅ 3️⃣ WebSocket 서버 실행 (GPS 전송)
    threading.Thread(target=lambda: asyncio.run(start_websocket_server()), daemon=True).start()

    # ✅ 4️⃣ Waypoints WebSocket 실행 (웹 → ROS)
    threading.Thread(target=lambda: asyncio.run(start_waypoints_websocket()), daemon=True).start()

    # ✅ 5️⃣ ROS 스핀 (노드 유지)
    rospy.spin()