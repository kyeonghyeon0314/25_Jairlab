#!/usr/bin/env python3

import rospy
import json
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
import tf2_ros
import tf2_geometry_msgs

class PathVisualizer:
    """경로 시각화 전용 클래스"""

    def __init__(self):
        rospy.init_node('path_visualizer', anonymous=True)

        # Waypoint 시각화만 활성화
        self.latest_waypoints = None

        # 시각화 Publishers (waypoint만)
        self.waypoints_pub = rospy.Publisher("/visualization/waypoints", MarkerArray, queue_size=10)

        # Subscribers (waypoint만)
        rospy.Subscriber("/waypoints", String, self.waypoints_callback)

        # Timers
        rospy.Timer(rospy.Duration(0.5), self.publish_visualization)

        rospy.loginfo("🎨 Waypoint 시각화기 시작!")
        rospy.loginfo("📍 Waypoint 마커 시각화")

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