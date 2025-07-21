#!/usr/bin/env python3

import rospy
import json
import utm
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class PathVisualizer:
    """
    경로 및 웨이포인트 시각화 전용 모듈
    - Corrected Path (빨간색): /fused_odom에서 보정된 로봇 궤적
    - GPS Path (파란색): /ublox/fix에서 UTM Local로 변환된 GPS 궤적  
    - Waypoints: 웹에서 받은 웨이포인트 시각화
    """
    # 색상 상수 정의
    COLORS = {
        'red': (1.0, 0.0, 0.0),
        'green': (0.0, 1.0, 0.0),
        'blue': (0.0, 0.0, 1.0),
        'yellow': (1.0, 1.0, 0.0),
        'orange': (1.0, 0.5, 0.0),
        'gray': (0.5, 0.5, 0.5),
        'white': (1.0, 1.0, 1.0)
    }
    
    def __init__(self):
        rospy.init_node('path_visualizer_node', anonymous=True)

        # 궤적 데이터
        self.gps_path = []
        self.corrected_path = []
        self.latest_waypoints = None

        # UTM 원점 정보 (initialize_pose.py에서 수신)
        self.utm_origin_absolute = None
        self.origin_synced = False

        # Publishers - 각 경로별 시각화
        self.gps_path_pub = rospy.Publisher("/gps_path", Marker, queue_size=1)
        self.corrected_path_pub = rospy.Publisher("/corrected_path", Marker, queue_size=1)
        self.waypoints_pub = rospy.Publisher("/global_waypoints", MarkerArray, queue_size=10)

        # Subscribers
        rospy.Subscriber("/utm_origin_info", String, self.utm_origin_callback)
        rospy.Subscriber("/fused_odom", Odometry, self.corrected_callback)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/waypoints", String, self.waypoints_callback)
        
        # 카카오 네비게이션 웨이포인트 시각화도 구독 (중복 제거용)
        rospy.Subscriber("/kakao_waypoints_viz", String, self.kakao_waypoints_callback)

        # Timer for periodic visualization update
        rospy.Timer(rospy.Duration(0.5), self.publish_all_visualizations)

        rospy.loginfo("📊 Path Visualizer 시작 - 경로 및 웨이포인트 시각화")

    def utm_origin_callback(self, msg):
        """UTM 원점 정보 동기화"""
        if not self.origin_synced:
            try:
                data = json.loads(msg.data)
                self.utm_origin_absolute = data["utm_origin_absolute"]
                self.origin_synced = True
                rospy.loginfo("✅ Path Visualizer UTM 원점 동기화 완료")
            except (json.JSONDecodeError, KeyError) as e:
                rospy.logerr(f"❌ UTM 원점 정보 파싱 실패: {e}")

    def corrected_callback(self, msg):
        """보정된 odometry 경로 (빨간색)"""
        point = Point()
        point.x = msg.pose.pose.position.x
        point.y = msg.pose.pose.position.y
        point.z = msg.pose.pose.position.z
        
        # 거리 기반 필터링 (0.5m 이상 이동시에만 기록)
        if not self.corrected_path or self.distance_check(point, self.corrected_path[-1], 0.5):
            self.corrected_path.append(point)
            
            # 궤적이 너무 길어지면 앞부분 제거 (최대 1000개 포인트)
            if len(self.corrected_path) > 1000:
                self.corrected_path = self.corrected_path[-800:]


    def gps_callback(self, msg):
        """GPS 경로 (파란색) - UTM Local로 변환"""
        if self.origin_synced and msg.status.status >= 0:
            try:
                lat, lon = msg.latitude, msg.longitude
                
                # GPS → UTM Local 변환
                if abs(lat) < 0.01 and abs(lon) < 0.01:
                    # 시뮬레이션 GPS
                    easting = lat * 111320.0
                    northing = lon * 111320.0
                else:
                    easting, northing, _, _ = utm.from_latlon(lat, lon)
                
                local_x = easting - self.utm_origin_absolute["easting"]
                local_y = northing - self.utm_origin_absolute["northing"]
                
                point = Point(x=local_x, y=local_y, z=0)
                
                # 거리 기반 필터링 (1m 이상 이동시에만 기록)
                if not self.gps_path or self.distance_check(point, self.gps_path[-1], 1.0):
                    self.gps_path.append(point)
                    
                    if len(self.gps_path) > 1000:
                        self.gps_path = self.gps_path[-800:]
                        
            except (KeyError, TypeError, ValueError) as e:
                rospy.logwarn_throttle(10, f"⚠️ GPS 변환 실패: {e}")

    def waypoints_callback(self, msg):
        """일반 웨이포인트 수신 (path_visualizer/waypoints_generator에서)"""
        try:
            data = json.loads(msg.data)
            
            # 카카오 네비게이션 데이터 필터링 (중복 방지)
            if "coordinate_type" in data and data["coordinate_type"] == "kakao_navigation_route":
                return  # 카카오 데이터는 별도 콜백에서 처리
            
            self.latest_waypoints = data
            rospy.logdebug("📍 일반 웨이포인트 수신됨")
            
        except json.JSONDecodeError as e:
            rospy.logwarn(f"⚠️ 웨이포인트 JSON 파싱 실패: {e}")

    def kakao_waypoints_callback(self, msg):
        """카카오 네비게이션 웨이포인트 시각화 (우선순위 높음)"""
        try:
            data = json.loads(msg.data)
            if data.get("coordinate_type") == "kakao_navigation_route":
                self.latest_waypoints = data
                rospy.loginfo_throttle(5, "🗺️ 카카오 네비게이션 웨이포인트 업데이트됨")
                
        except json.JSONDecodeError as e:
            rospy.logwarn(f"⚠️ 카카오 웨이포인트 파싱 실패: {e}")

    def publish_all_visualizations(self, _):
        """모든 시각화 정기적 발행"""
        self.publish_gps_path()
        self.publish_corrected_path()
        self.publish_waypoints()

    def publish_gps_path(self):
        """GPS 경로 시각화 (파란색)"""
        if len(self.gps_path) < 2:
            return
        
        marker = self.create_path_marker(
            self.gps_path, "gps_path", "utm_local",
            (0.0, 0.0, 1.0), 3.0  # 파란색, 굵게
        )
        self.gps_path_pub.publish(marker)

    def publish_corrected_path(self):
        """보정된 경로 시각화 (빨간색)"""
        if len(self.corrected_path) < 2:
            return
        
        marker = self.create_path_marker(
            self.corrected_path, "corrected_path", "utm_local",
            (1.0, 0.0, 0.0), 3.0  # 빨간색, 굵게
        )
        self.corrected_path_pub.publish(marker)


    def publish_waypoints(self):
        """웨이포인트 시각화"""
        marker_array = MarkerArray()
        
        # 기존 마커 삭제
        delete_marker = Marker()
        delete_marker.header.frame_id = "utm_local"
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        if not self.latest_waypoints or "waypoints" not in self.latest_waypoints:
            self.waypoints_pub.publish(marker_array)
            return
        
        waypoints = self.latest_waypoints["waypoints"]
        current_wp_index = self.latest_waypoints.get("current_waypoint", 0)
        is_kakao_navigation = self.latest_waypoints.get("coordinate_type") == "kakao_navigation_route"
        
        if not waypoints:
            self.waypoints_pub.publish(marker_array)
            return
        
        valid_points = []
        valid_waypoints = []
        
        # 유효한 웨이포인트 수집
        for i, wp in enumerate(waypoints):
            if "x" in wp and "y" in wp:
                valid_points.append(Point(x=float(wp["x"]), y=float(wp["y"]), z=0))
                valid_waypoints.append((i, float(wp["x"]), float(wp["y"]), wp))
        
        if not valid_waypoints:
            self.waypoints_pub.publish(marker_array)
            return
        
        # 연결선 마커 (궤적)
        if len(valid_points) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = "utm_local"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "waypoints_path"
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 1.5
            
            # 카카오 네비게이션은 주황색, 일반은 노란색
            color = self.COLORS['orange'] if is_kakao_navigation else self.COLORS['yellow']
            line_marker.color.r, line_marker.color.g, line_marker.color.b = color
            
            line_marker.color.a = 1.0
            line_marker.pose.orientation.w = 1.0
            line_marker.points = valid_points
            marker_array.markers.append(line_marker)
        
        # 개별 웨이포인트 마커들
        for wp_index, (original_index, x, y, wp_data) in enumerate(valid_waypoints):
            # 웨이포인트 큐브
            cube = Marker()
            cube.header.frame_id = "utm_local"
            cube.header.stamp = rospy.Time.now()
            cube.ns = "waypoints_cubes"
            cube.id = wp_index + 1
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = x
            cube.pose.position.y = y
            cube.pose.position.z = 2.0
            cube.pose.orientation.w = 1.0
            cube.scale.x = 4.0
            cube.scale.y = 4.0
            cube.scale.z = 2.5
            
            # 상태에 따른 색상 결정
            if is_kakao_navigation:
                if wp_data.get("completed", False):
                    color = self.COLORS['gray']
                elif wp_data.get("is_current", False):
                    color = self.COLORS['green']
                elif wp_data.get("is_destination", False):
                    color = self.COLORS['red']
                else:
                    color = self.COLORS['orange']
            else:
                if original_index < current_wp_index:
                    color = self.COLORS['gray']
                elif original_index == current_wp_index:
                    color = self.COLORS['green']
                elif original_index == 0:
                    color = self.COLORS['green']
                elif original_index == len(waypoints) - 1:
                    color = self.COLORS['red']
                else:
                    color = self.COLORS['yellow']
            
            cube.color.r, cube.color.g, cube.color.b = color
            
            cube.color.a = 0.9
            marker_array.markers.append(cube)
            
            # 웨이포인트 번호 텍스트
            text = Marker()
            text.header.frame_id = "utm_local"
            text.header.stamp = rospy.Time.now()
            text.ns = "waypoints_text"
            text.id = wp_index
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 6.0
            text.pose.orientation.w = 1.0
            text.scale.z = 4.0
            text.color.r, text.color.g, text.color.b = self.COLORS['white']
            text.color.a = 1.0
            
            # 텍스트 내용
            if is_kakao_navigation:
                if wp_data.get("is_destination", False):
                    text.text = f"목적지"
                else:
                    text.text = f"WP{original_index+1}"
            else:
                text.text = f"WP{original_index+1}"
            
            marker_array.markers.append(text)
        
        # 마커 발행
        self.waypoints_pub.publish(marker_array)

    def create_path_marker(self, path_points, namespace, frame_id, color, line_width):
        """경로 마커 생성 헬퍼"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = line_width
        marker.color.r, marker.color.g, marker.color.b = color[0], color[1], color[2]
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.points = path_points
        return marker

    def distance_check(self, p1, p2, threshold):
        """두 점 사이의 거리가 임계값보다 큰지 확인"""
        if isinstance(p1, Point):
            x1, y1 = p1.x, p1.y
        else:
            x1, y1 = p1["x"], p1["y"]
            
        if isinstance(p2, Point):
            x2, y2 = p2.x, p2.y
        else:
            x2, y2 = p2["x"], p2["y"]
            
        distance = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
        return distance > threshold

if __name__ == '__main__':
    try:
        PathVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Path Visualizer 종료")