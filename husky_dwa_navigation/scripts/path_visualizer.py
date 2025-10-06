#!/usr/bin/env python3

import rospy
import json
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import utm
from sensor_msgs.msg import NavSatFix
import tf2_ros
import tf2_geometry_msgs

class PathVisualizer:
    """경로 시각화 전용 클래스"""

    def __init__(self):
        rospy.init_node('path_visualizer', anonymous=True)

        # 🎯 UTM 원점(datum) 관리
        self.utm_origin_absolute = None
        self.utm_zone = None
        self.origin_published = False
        self.first_gps = None

        # 웨이포인트 시각화
        self.latest_waypoints = None

        # Publishers
        self.waypoints_pub = rospy.Publisher("/visualization/waypoints", MarkerArray, queue_size=10)
        self.utm_origin_pub = rospy.Publisher("/utm_origin_info", String, queue_size=1, latch=True)

        # Subscribers
        # 🛰️ robot_localization의 첫 odom을 기준으로 datum 설정
        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback, queue_size=1)
        self.gps_sub = rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback, queue_size=1)
        rospy.Subscriber("/waypoints", String, self.waypoints_callback)
        rospy.Subscriber("/kakao_waypoints_viz", String, self.waypoints_callback)

        # Timers
        rospy.Timer(rospy.Duration(0.5), self.publish_visualization)

        rospy.loginfo("🎨 경로 시각화기 시작!")
        rospy.loginfo("🛰️ GPS 및 Odometry 수신 대기 중 (UTM 원점 설정을 위해)...")

    def gps_callback(self, msg):
        """첫 GPS 데이터를 저장하여 datum 설정에 사용"""
        if self.first_gps is None and msg.status.status >= 0: # 유효한 GPS fix만 사용
            self.first_gps = msg
            rospy.loginfo(f"🛰️ 첫 유효 GPS 수신: Lat {msg.latitude:.6f}, Lon {msg.longitude:.6f}")
            # GPS 수신 후에는 더 이상 이 콜백이 필요 없으므로 구독 해제
            self.gps_sub.unregister()
            rospy.loginfo("✅ GPS 구독 해제. Odometry 대기 중...")

    def odom_callback(self, msg):
        """첫 Odometry를 기준으로 UTM 원점(datum)을 설정하고 발행"""
        if self.origin_published:
            # 원점이 이미 발행되었으면 odom 구독도 중지하여 리소스 절약
            if self.odom_sub:
                self.odom_sub.unregister()
                self.odom_sub = None
            return

        if self.first_gps is None:
            rospy.loginfo_throttle(10, "⏳ Odometry 수신 중... 첫 GPS 신호를 기다립니다.")
            return

        try:
            # 첫 GPS 좌표를 절대 UTM으로 변환
            lat = self.first_gps.latitude
            lon = self.first_gps.longitude
            easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)

            # Odometry의 첫 위치는 (0,0)이므로, 이 위치의 절대 UTM 좌표가 바로 원점이 됨
            self.utm_origin_absolute = {
                "easting": easting,
                "northing": northing,
                "lat": lat,
                "lon": lon
            }
            self.utm_zone = f"{zone_num}{zone_letter}"

            # 원점 정보를 JSON 문자열로 만들어 발행 (latch=True)
            origin_info = {"utm_origin_absolute": self.utm_origin_absolute, "utm_zone": self.utm_zone}
            self.utm_origin_pub.publish(json.dumps(origin_info))
            self.origin_published = True

            rospy.loginfo("="*50)
            rospy.loginfo(f"🎯 UTM 원점(datum) 설정 및 발행 완료! (기준: 첫 GPS)")
            rospy.loginfo(f"   - 절대좌표: Easting={easting:.2f}, Northing={northing:.2f}, Zone={self.utm_zone}")
            rospy.loginfo("="*50)

        except Exception as e:
            rospy.logerr(f"❌ UTM 원점 설정 중 오류 발생: {e}")

    def distance_check(self, pose1, pose2, threshold):
        """거리 체크"""
        if "x" not in pose1 or "x" not in pose2:
            return True
        dx = pose1["x"] - pose2["x"]
        dy = pose1["y"] - pose2["y"]
        return math.sqrt(dx*dx + dy*dy) > threshold

    def waypoints_callback(self, msg):
        """Waypoints 수신"""
        try:
            data = json.loads(msg.data)
            if "waypoints" in data:
                self.latest_waypoints = data["waypoints"]
                rospy.loginfo(f"📥 {len(self.latest_waypoints)}개 waypoints 수신")
                self.visualize_waypoints()
        except Exception as e:
            rospy.logerr(f"❌ Waypoints 파싱 오류: {e}")

    def publish_visualization(self, event):
        """Waypoint 시각화만 발행"""
        self.visualize_waypoints()

    def visualize_waypoints(self):
        """웨이포인트 시각화"""
        marker_array = MarkerArray()

        # 기존 마커 삭제
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = "waypoints"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        if not self.latest_waypoints:
            self.waypoints_pub.publish(marker_array)
            return

        # 웨이포인트 마커들 생성
        for i, wp in enumerate(self.latest_waypoints):
            if "x" in wp and "y" in wp:
                # UTM 좌표 웨이포인트
                cube = Marker()
                cube.header.frame_id = "map"
                cube.header.stamp = rospy.Time.now()
                cube.ns = "waypoints"
                cube.id = i
                cube.type = Marker.CUBE
                cube.action = Marker.ADD
                cube.pose.position.x = float(wp["x"])
                cube.pose.position.y = float(wp["y"])
                cube.pose.position.z = 2.0
                cube.pose.orientation.w = 1.0
                cube.scale.x = 4.0
                cube.scale.y = 4.0
                cube.scale.z = 2.5

                # 색상 구분
                if i == 0:
                    cube.color.r, cube.color.g, cube.color.b = 0.0, 1.0, 0.0  # 시작점 - 녹색
                elif i == len(self.latest_waypoints) - 1:
                    cube.color.r, cube.color.g, cube.color.b = 1.0, 0.0, 0.0  # 끝점 - 빨간색
                else:
                    cube.color.r, cube.color.g, cube.color.b = 1.0, 1.0, 0.0  # 중간점 - 노란색

                cube.color.a = 0.9
                marker_array.markers.append(cube)

        self.waypoints_pub.publish(marker_array)

    def create_path_marker(self, trajectory, namespace, marker_id, color, line_width, frame_id):
        """경로 마커 생성"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = line_width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0

        points = []
        for pt in trajectory:
            if "x" in pt and "y" in pt:
                points.append(Point(x=pt["x"], y=pt["y"], z=pt.get("z", 0)))

        marker.points = points
        return marker

if __name__ == '__main__':
    try:
        visualizer = PathVisualizer()
        rospy.loginfo("🎉 Waypoint 시각화기 실행 중...")
        rospy.loginfo("📍 시각화 토픽: /visualization/waypoints")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 시스템 종료")
    except Exception as e:
        rospy.logerr(f"❌ 시스템 오류: {e}")