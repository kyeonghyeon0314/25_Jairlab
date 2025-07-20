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
import utm
from std_msgs.msg import String
from nav_msgs.msg import Odometry

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
utm_origin_absolute = None
utm_zone = None
realtime_gps = None
system_status = {
    "utm_origin_synced": False,
    "fasterlio_active": False,
    "gps_active": False,
    "navigation_active": False
}

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
    
    def log_message(self, format, *args):
        # HTTP 로그 억제 (너무 많은 로그 방지)
        pass

def start_http_server():
    """ HTTP 서버 실행 (index.html 제공) """
    try:
        os.chdir(WEB_DIR)
        with socketserver.TCPServer(("", PORT), CustomHandler) as httpd:
            rospy.loginfo(f"🌍 HTTP 서버 실행 중: http://localhost:{PORT}")
            httpd.serve_forever()
    except OSError as e:
        rospy.logerr(f"❌ HTTP 서버 포트({PORT}) 이미 사용 중! 기존 프로세스를 종료하세요: {e}")
    except Exception as e:
        rospy.logerr(f"❌ HTTP 서버 시작 실패: {e}")

def open_browser():
    """ 웹 브라우저 자동 실행 """
    url = f"http://localhost:{PORT}/index.html"
    rospy.loginfo(f"🌐 브라우저 열기: {url}")
    try:
        webbrowser.open(url)
    except Exception as e:
        rospy.logwarn(f"⚠️ 브라우저 자동 실행 실패: {e}")

# ---------------------------
# 📌 ROS 노드 설정 (GPS 데이터 수신)
# ---------------------------
def gps_callback(data):
    """ ROS에서 GPS 데이터 수신 후 저장 """
    global latest_gps_data, system_status
    try:
        with data_lock:
            latest_gps_data = json.loads(data.data)  # 문자열을 JSON으로 변환
            system_status["gps_active"] = True
        rospy.loginfo_throttle(15, f"📡 ROS GPS 데이터 수신: {latest_gps_data}")
    except json.JSONDecodeError as e:
        rospy.logwarn(f"⚠️ GPS 데이터 JSON 파싱 실패: {e}")
    except Exception as e:
        rospy.logerr(f"❌ GPS 콜백 오류: {e}")

def utm_origin_callback(msg):
    """UTM 원점 정보 동기화 (make_frame.py에서 발행)"""
    global utm_origin_absolute, utm_zone, system_status
    try:
        data = json.loads(msg.data)
        utm_origin_absolute = data["utm_origin_absolute"]
        utm_zone = data["utm_zone"]
        system_status["utm_origin_synced"] = True
        rospy.loginfo(f"✅ UTM 원점 동기화 완료: Zone {utm_zone}")
    except (json.JSONDecodeError, KeyError) as e:
        rospy.logwarn(f"⚠️ UTM 원점 데이터 파싱 실패: {e}")

def navigation_status_callback(msg):
    """네비게이션 상태 모니터링"""
    global system_status
    try:
        data = json.loads(msg.data)
        system_status["navigation_active"] = data.get("navigation_active", False)
    except (json.JSONDecodeError, KeyError):
        pass

def fasterlio_callback(msg):
    """FasterLIO 상태 모니터링"""
    global system_status
    system_status["fasterlio_active"] = True

def start_ros_node():
    """ ROS 노드 초기화 및 구독 (메인 스레드에서 실행) """
    rospy.init_node(ROS_NODE_NAME, anonymous=True)
    
    # 기존 GPS 토픽 구독
    rospy.Subscriber(GPS_TOPIC, String, gps_callback)
    
    # 시스템 상태 모니터링 구독
    rospy.Subscriber("/utm_origin_info", String, utm_origin_callback)
    rospy.Subscriber("/navigation/web_status", String, navigation_status_callback)
    rospy.Subscriber("/Odometry", Odometry, fasterlio_callback)  # FasterLIO 활성 상태 확인
    
    rospy.loginfo(f"🚀 ROS 노드 '{ROS_NODE_NAME}' 실행 완료")

# ---------------------------
# 📌 WebSocket 서버 실행 (GPS 데이터 전송)
# ---------------------------
async def send_gps_data(websocket, path):
    """ WebSocket을 통해 웹 클라이언트로 GPS 데이터 전송 """
    client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
    rospy.loginfo(f"📡 WebSocket GPS 클라이언트 연결: {client_info}")
    
    try:
        while True:
            with data_lock:
                # 실시간 GPS 우선, 없으면 기본 GPS 데이터
                data_to_send = realtime_gps or latest_gps_data
                current_system_status = system_status.copy()
                
            if data_to_send:
                # GPS 데이터에 시스템 상태 추가
                enhanced_data = data_to_send.copy()
                enhanced_data.update({
                    "system_status": current_system_status,
                    "timestamp": time.time(),
                    "utm_zone": utm_zone,
                    "server_info": {
                        "utm_origin_synced": utm_origin_absolute is not None,
                        "coordinate_system": "utm_local" if utm_origin_absolute else "unknown"
                    }
                })
                gps_data = json.dumps(enhanced_data)
            else:
                # GPS 데이터가 없어도 시스템 상태는 전송
                fallback_data = {
                    "error": "GPS 데이터 없음",
                    "system_status": current_system_status,
                    "timestamp": time.time(),
                    "utm_zone": utm_zone,
                    "server_info": {
                        "utm_origin_synced": utm_origin_absolute is not None,
                        "coordinate_system": "utm_local" if utm_origin_absolute else "unknown"
                    }
                }
                gps_data = json.dumps(fallback_data)
                
            await websocket.send(gps_data)
            rospy.loginfo_throttle(20, f"📡 WebSocket GPS 전송: {len(gps_data)} bytes → {client_info}")
            
            await asyncio.sleep(1)
            
    except websockets.exceptions.ConnectionClosed:
        rospy.loginfo(f"📡 WebSocket GPS 클라이언트 연결 해제: {client_info}")
    except Exception as e:
        rospy.logwarn(f"⚠️ WebSocket GPS 전송 오류 ({client_info}): {e}")

async def start_websocket_server():
    """ WebSocket 서버 실행 """
    try:
        rospy.loginfo(f"🔗 WebSocket GPS 서버 실행 중: ws://localhost:{WEBSOCKET_PORT}")
        async with websockets.serve(send_gps_data, "localhost", WEBSOCKET_PORT):
            await asyncio.Future()  # 무한 대기
    except OSError as e:
        rospy.logerr(f"❌ WebSocket 포트({WEBSOCKET_PORT}) 이미 사용 중! 기존 프로세스를 종료하세요: {e}")
    except Exception as e:
        rospy.logerr(f"❌ WebSocket 서버 시작 실패: {e}")

# ---------------------------
# 📌 WebSocket (웹 → ROS로 Waypoints 전송) - 개선된 GPS 변환
# ---------------------------
def gps_to_utm_local(lat, lon):
    """GPS → UTM Local 변환 (견고성 개선)"""
    if utm_origin_absolute is None:
        rospy.logwarn("⚠️ UTM 원점이 설정되지 않음, GPS 변환 불가")
        return None, None
        
    try:
        # 시뮬레이션 GPS 처리 (Gazebo 등)
        if abs(lat) < 0.01 and abs(lon) < 0.01:
            easting = lat * 111320.0  # 대략적인 미터 변환
            northing = lon * 111320.0
        else:
            # 실제 GPS 좌표 처리
            easting, northing, _, _ = utm.from_latlon(lat, lon)
            
        # UTM Local 상대좌표 계산
        local_x = easting - utm_origin_absolute["easting"]
        local_y = northing - utm_origin_absolute["northing"]
        
        return local_x, local_y
        
    except Exception as e:
        rospy.logerr(f"❌ GPS 좌표 변환 실패 ({lat}, {lon}): {e}")
        return None, None

async def receive_waypoints(websocket, path):
    """ 웹에서 받은 경로 데이터를 강화된 UTM Local 변환 후 ROS 토픽으로 전송 """
    global latest_waypoints
    pub = rospy.Publisher(WAYPOINTS_TOPIC, String, queue_size=10)
    
    client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
    rospy.loginfo(f"🔗 WebSocket 웨이포인트 클라이언트 연결: {client_info}")

    try:
        async for message in websocket:
            try:
                waypoints_data = json.loads(message)
                
                rospy.loginfo(f"📥 웨이포인트 수신 ({client_info}): {len(message)} bytes")
                
                if "waypoints" not in waypoints_data:
                    error_msg = "잘못된 웨이포인트 형식: 'waypoints' 키 없음"
                    rospy.logerr(f"❌ {error_msg}")
                    await websocket.send(json.dumps({"error": error_msg}))
                    continue

                # UTM 원점 확인
                if utm_origin_absolute is None:
                    error_msg = "UTM 원점이 설정되지 않음. make_frame 노드를 먼저 실행하세요."
                    rospy.logerr(f"❌ {error_msg}")
                    await websocket.send(json.dumps({"error": error_msg}))
                    continue

                # 웨이포인트 변환 처리
                converted_waypoints = []
                conversion_errors = []
                
                for i, wp in enumerate(waypoints_data["waypoints"]):
                    if "lat" not in wp or "lon" not in wp:
                        error_detail = f"웨이포인트 {i+1}: GPS 좌표 누락 (lat/lon)"
                        rospy.logwarn(f"⚠️ {error_detail}")
                        conversion_errors.append({"index": i, "error": error_detail})
                        continue
                    
                    # GPS 좌표 유효성 검사
                    try:
                        lat, lon = float(wp["lat"]), float(wp["lon"])
                        if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
                            error_detail = f"웨이포인트 {i+1}: GPS 좌표 범위 오류 (lat: {lat}, lon: {lon})"
                            rospy.logwarn(f"⚠️ {error_detail}")
                            conversion_errors.append({"index": i, "error": error_detail})
                            continue
                    except (ValueError, TypeError):
                        error_detail = f"웨이포인트 {i+1}: GPS 좌표 형식 오류"
                        conversion_errors.append({"index": i, "error": error_detail})
                        continue
                        
                    # UTM Local 변환
                    local_x, local_y = gps_to_utm_local(lat, lon)
                    if local_x is not None and local_y is not None:
                        converted_wp = {
                            "x": local_x, 
                            "y": local_y,
                            "index": i,
                            "original_gps": {"lat": lat, "lon": lon}
                        }
                        
                        # 추가 속성 보존 (속도, 방향, 정지시간 등)
                        for key in ["speed", "heading", "stop_time", "waypoint_type", "description"]:
                            if key in wp:
                                converted_wp[key] = wp[key]
                        
                        converted_waypoints.append(converted_wp)
                        
                        # 상세 로깅 (처음 3개와 마지막 3개만)
                        total_wps = len(waypoints_data["waypoints"])
                        if i < 3 or i >= total_wps - 3:
                            rospy.loginfo(f"   WP{i+1}: GPS({lat:.6f}, {lon:.6f}) → Local({local_x:.1f}, {local_y:.1f})")
                        elif i == 3 and total_wps > 6:
                            rospy.loginfo(f"   ... (중간 {total_wps-6}개 웨이포인트 생략)")
                    else:
                        conversion_errors.append({"index": i, "error": "UTM 변환 실패"})

                # 목적지 변환 (있는 경우)
                destination_converted = None
                destination_error = None
                if "destination" in waypoints_data and waypoints_data["destination"]:
                    dest = waypoints_data["destination"]
                    if "lat" in dest and "lon" in dest:
                        try:
                            dest_lat, dest_lon = float(dest["lat"]), float(dest["lon"])
                            dest_x, dest_y = gps_to_utm_local(dest_lat, dest_lon)
                            if dest_x is not None and dest_y is not None:
                                destination_converted = {
                                    "x": dest_x, 
                                    "y": dest_y,
                                    "original_gps": {"lat": dest_lat, "lon": dest_lon}
                                }
                                rospy.loginfo(f"   🎯 목적지: GPS({dest_lat:.6f}, {dest_lon:.6f}) → Local({dest_x:.1f}, {dest_y:.1f})")
                            else:
                                destination_error = "목적지 UTM 변환 실패"
                        except (ValueError, TypeError):
                            destination_error = "목적지 GPS 좌표 형식 오류"
                    else:
                        destination_error = "목적지 GPS 좌표 누락"
                
                if destination_error:
                    rospy.logwarn(f"⚠️ {destination_error}")

                # 변환된 데이터 구성 (강화된 메타데이터 포함)
                converted_data = {
                    "waypoints": converted_waypoints,
                    "destination": destination_converted,
                    "coordinate_system": "utm_local",
                    "utm_zone": utm_zone,
                    "conversion_stats": {
                        "total_received": len(waypoints_data["waypoints"]),
                        "successfully_converted": len(converted_waypoints),
                        "conversion_errors": len(conversion_errors),
                        "error_details": conversion_errors
                    },
                    "metadata": {
                        "source": "web_interface",
                        "timestamp": time.time(),
                        "utm_origin": utm_origin_absolute,
                        "destination_error": destination_error
                    }
                }

                # ROS 토픽으로 발행
                pub.publish(json.dumps(converted_data))
                
                # 성공 응답 (상세 정보 포함)
                response = {
                    "status": "success",
                    "converted_waypoints": len(converted_waypoints),
                    "total_received": len(waypoints_data["waypoints"]),
                    "conversion_errors": len(conversion_errors),
                    "destination_converted": destination_converted is not None,
                    "destination_error": destination_error,
                    "utm_zone": utm_zone,
                    "coordinate_system": "utm_local"
                }
                
                rospy.loginfo(f"✅ 웨이포인트 변환 완료: {len(converted_waypoints)}/{len(waypoints_data['waypoints'])}개")
                
                if conversion_errors:
                    rospy.logwarn(f"⚠️ {len(conversion_errors)}개 웨이포인트 변환 실패")
                    response["error_details"] = conversion_errors
                    
                await websocket.send(json.dumps(response))

            except json.JSONDecodeError as e:
                error_msg = f"JSON 파싱 오류: {e}"
                rospy.logerr(f"❌ {error_msg}")
                await websocket.send(json.dumps({"error": error_msg}))
                
            except Exception as e:
                error_msg = f"웨이포인트 처리 오류: {e}"
                rospy.logerr(f"❌ {error_msg}")
                try:
                    await websocket.send(json.dumps({"error": error_msg}))
                except:
                    pass
                    
    except websockets.exceptions.ConnectionClosed:
        rospy.loginfo(f"📡 웨이포인트 WebSocket 클라이언트 연결 해제: {client_info}")
    except Exception as e:
        rospy.logerr(f"❌ 웨이포인트 WebSocket 오류 ({client_info}): {e}")

async def start_waypoints_websocket():
    """ Waypoints WebSocket 서버 실행 """
    try:
        rospy.loginfo(f"🔗 Waypoints WebSocket 실행 중: ws://localhost:{WAYPOINTS_WEBSOCKET_PORT}")
        async with websockets.serve(receive_waypoints, "localhost", WAYPOINTS_WEBSOCKET_PORT):
            await asyncio.Future()
    except OSError as e:
        rospy.logerr(f"❌ Waypoints WebSocket 포트({WAYPOINTS_WEBSOCKET_PORT}) 이미 사용 중! 기존 프로세스를 종료하세요: {e}")
    except Exception as e:
        rospy.logerr(f"❌ Waypoints WebSocket 서버 시작 실패: {e}")

# ---------------------------
# 📌 상태 모니터링 및 로깅
# ---------------------------
def status_monitor():
    """시스템 상태 주기적 모니터링 및 진단"""
    consecutive_errors = 0
    
    while not rospy.is_shutdown():
        try:
            with data_lock:
                gps_available = latest_gps_data is not None
                realtime_gps_available = realtime_gps is not None
                utm_synced = utm_origin_absolute is not None
                current_status = system_status.copy()
            
            # 상태 진단
            issues = []
            if not utm_synced:
                issues.append("UTM 원점 미동기화")
            if not current_status["fasterlio_active"]:
                issues.append("FasterLIO 비활성")
            if not current_status["gps_active"]:
                issues.append("GPS 비활성")
                
            # 주기적 상태 로깅
            if issues:
                rospy.logwarn_throttle(60, f"⚠️ 시스템 이슈: {', '.join(issues)}")
                consecutive_errors += 1
            else:
                if consecutive_errors > 0:
                    rospy.loginfo("✅ 모든 시스템 이슈 해결됨")
                    consecutive_errors = 0
                rospy.loginfo_throttle(120, f"✅ 시스템 정상: GPS={gps_available}, UTM동기화={utm_synced}, 네비게이션={current_status['navigation_active']}")
            
            # 심각한 오류 시 경고
            if consecutive_errors > 10:
                rospy.logerr("❌ 심각한 시스템 문제 감지! 노드들을 재시작하는 것을 고려하세요.")
                consecutive_errors = 0  # 스팸 방지
                
        except Exception as e:
            rospy.logwarn(f"⚠️ 상태 모니터링 오류: {e}")
            
        time.sleep(15)  # 15초마다 체크

def system_info_publisher():
    """시스템 정보 주기적 발행"""
    info_pub = rospy.Publisher("/gps_server/system_info", String, queue_size=1, latch=True)
    
    while not rospy.is_shutdown():
        try:
            with data_lock:
                current_status = system_status.copy()
                
            system_info = {
                "node": ROS_NODE_NAME,
                "version": "2.0",
                "status": current_status,
                "services": {
                    "http_server": f"http://localhost:{PORT}",
                    "gps_websocket": f"ws://localhost:{WEBSOCKET_PORT}",
                    "waypoints_websocket": f"ws://localhost:{WAYPOINTS_WEBSOCKET_PORT}"
                },
                "utm_info": {
                    "zone": utm_zone,
                    "origin_synced": utm_origin_absolute is not None
                },
                "timestamp": time.time()
            }
            
            info_pub.publish(json.dumps(system_info))
            
        except Exception as e:
            rospy.logwarn(f"⚠️ 시스템 정보 발행 오류: {e}")
            
        time.sleep(30)  # 30초마다 발행

# ---------------------------
# 📌 메인 실행부
# ---------------------------
if __name__ == '__main__':
    try:
        rospy.loginfo("🚀 GPS 서버 시작 중...")
        
        # ✅ 1️⃣ ROS 노드 실행 (메인 스레드에서 실행)
        start_ros_node()

        # ✅ 2️⃣ HTTP 서버 실행 (웹 제공)
        threading.Thread(target=start_http_server, daemon=True).start()
        
        # ✅ 3️⃣ 상태 모니터링 스레드들
        threading.Thread(target=status_monitor, daemon=True).start()
        threading.Thread(target=system_info_publisher, daemon=True).start()
        
        # 서버 안정화 대기
        time.sleep(3)
        open_browser()  # 🌍 웹페이지 자동 실행

        # ✅ 4️⃣ WebSocket 서버 실행 (GPS 전송)
        threading.Thread(target=lambda: asyncio.run(start_websocket_server()), daemon=True).start()

        # ✅ 5️⃣ Waypoints WebSocket 실행 (웹 → ROS)
        threading.Thread(target=lambda: asyncio.run(start_waypoints_websocket()), daemon=True).start()

        rospy.loginfo("🎉 GPS 서버 모든 구성요소 시작 완료!")
        rospy.loginfo(f"   - 🌍 웹 인터페이스: http://localhost:{PORT}")
        rospy.loginfo(f"   - 📡 GPS WebSocket: ws://localhost:{WEBSOCKET_PORT}")
        rospy.loginfo(f"   - 🗺️ Waypoints WebSocket: ws://localhost:{WAYPOINTS_WEBSOCKET_PORT}")
        rospy.loginfo(f"   - 🎯 좌표계: UTM Local (원점 대기 중)")

        # ✅ 6️⃣ ROS 스핀 (노드 유지)
        rospy.spin()
        
    except KeyboardInterrupt:
        rospy.loginfo("🛑 사용자에 의한 GPS 서버 종료")
    except Exception as e:
        rospy.logerr(f"❌ GPS 서버 시작 실패: {e}")
    finally:
        rospy.loginfo("🔚 GPS 서버 종료")