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
import utm  # UTM 변환을 위한 라이브러리 추가
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

# 📌 HTTP & WebSocket 설정
PORT = 8000
WEBSOCKET_PORT = 8765
WAYPOINTS_WEBSOCKET_PORT = 8766
WEB_DIR = os.path.join(os.path.dirname(__file__), "../web")  # 웹 폴더 경로

# 📌 ROS 설정
ROS_NODE_NAME = "gps_server"
GPS_TOPIC = "/ublox/fix"
WAYPOINTS_TOPIC = "waypoints"

# 최신 GPS 데이터 및 Waypoints 저장 (쓰레드 안전)
latest_gps_data = None
latest_waypoints = None
data_lock = threading.Lock()
string_pub = None  # compatibility publisher for legacy String-based GPS topic

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
    rospy.loginfo_throttle(15, f"📡 ROS GPS 데이터 수신: {latest_gps_data}")


def ublox_callback(msg):
    """NavSatFix 콜백: /ublox/fix에서 수신한 위도/경도 정보를 웹으로 보낼 수 있게 저장"""
    global latest_gps_data
    try:
        lat = getattr(msg, 'latitude', None)
        lon = getattr(msg, 'longitude', None)
        alt = getattr(msg, 'altitude', None) if hasattr(msg, 'altitude') else None

        if lat is None or lon is None:
            rospy.logwarn_throttle(30, "⚠️ /ublox/fix: 위도/경도 정보 없음")
            return

        gps_dict = {
            "lat": float(lat),
            "lon": float(lon),
            "alt": float(alt) if alt is not None else None,
            # also include standard field names expected by the web client
            "latitude": float(lat),
            "longitude": float(lon),
            "altitude": float(alt) if alt is not None else None,
            "stamp": msg.header.stamp.to_sec() if hasattr(msg, 'header') else time.time()
        }

        with data_lock:
            latest_gps_data = gps_dict

        # Legacy compatibility: publish JSON string on /gps_data for nodes expecting std_msgs/String
        try:
            if string_pub is not None:
                string_pub.publish(String(data=json.dumps(gps_dict)))
        except Exception:
            pass

        rospy.loginfo_throttle(10, f"📡 /ublox/fix 수신: lat={lat}, lon={lon}")

    except Exception as e:
        rospy.logwarn(f"⚠️ ublox_callback 처리 중 오류: {e}")

def start_ros_node():
    """ ROS 노드 초기화 및 구독 (메인 스레드에서 실행) """
    # 고정된 노드 이름으로 실행해 디버깅 편의성을 높임
    rospy.init_node(ROS_NODE_NAME, anonymous=False)

    # /ublox/fix는 sensor_msgs/NavSatFix 타입을 사용하므로 해당 타입으로 구독
    rospy.Subscriber(GPS_TOPIC, NavSatFix, ublox_callback)
    # Legacy: publish a JSON string version so older nodes expecting std_msgs/String still work
    global string_pub
    string_pub = rospy.Publisher('/gps_data', String, queue_size=10)
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
        # 로그 스팸 방지: 실제 전송은 초당 유지하되 터미널 출력은 throttle
        rospy.loginfo_throttle(20, f"📡 WebSocket 전송 (요약): {len(gps_data)} bytes")

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
            waypoints_data = json.loads(message)
            if not isinstance(waypoints_data, dict) or "waypoints" not in waypoints_data:
                rospy.logerr("❌ 잘못된 Waypoints 데이터 형식!")
                continue

            # UTM 변환 로직 추가
            converted_waypoints = []
            for wp in waypoints_data.get("waypoints", []):
                if "lat" in wp and "lon" in wp:
                    try:
                        utm_x, utm_y, _, _ = utm.from_latlon(wp["lat"], wp["lon"])
                        converted_wp = {
                            "x": utm_x,
                            "y": utm_y,
                            "original_gps": {"lat": wp["lat"], "lon": wp["lon"]}
                        }
                        converted_waypoints.append(converted_wp)
                    except Exception as e:
                        rospy.logwarn(f"⚠️ 웨이포인트 UTM 변환 실패: {wp}, 오류: {e}")
                else:
                    converted_waypoints.append(wp) # lat/lon 없는 데이터는 그대로 추가

            # 변환된 데이터로 업데이트
            final_data_to_publish = {
                "waypoints": converted_waypoints,
                "destination": waypoints_data.get("destination") # 목적지 정보 유지
            }

            with data_lock:
                latest_waypoints = final_data_to_publish

            pub.publish(json.dumps(final_data_to_publish))
            rospy.loginfo(f"🗺️ UTM 변환된 Waypoints 발행: {len(converted_waypoints)}개")

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