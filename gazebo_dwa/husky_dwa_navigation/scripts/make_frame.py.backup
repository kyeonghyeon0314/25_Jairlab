#!/usr/bin/env python3
import rospy
import utm
import json
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros
from geometry_msgs.msg import TransformStamped

class FrameMaker:
    """UTM Local 좌표계 설정 및 GPS-FasterLIO 동기화"""
    
    def __init__(self):
        rospy.init_node('make_frame_node', anonymous=True)

        # UTM 원점 관리
        self.utm_origin_absolute = None
        self.utm_zone = None
        self.first_gps_received = False
        
        # FasterLIO 관리
        self.fasterlio_origin = None
        self.current_body_pose = None
        
        # 실시간 GPS 데이터 (웹 전송용)
        self.current_gps = None

        # Publishers
        self.utm_origin_pub = rospy.Publisher("/utm_origin_info", String, queue_size=1, latch=True)
        self.gps_data_pub = rospy.Publisher("/gps_data", String, queue_size=10)  # 웹 전송용
        
        # Subscribers
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/Odometry", Odometry, self.fasterlio_callback)

        # Static TF broadcaster
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # Timer for GPS data publishing
        rospy.Timer(rospy.Duration(1.0), self.publish_gps_data)

        rospy.loginfo("🎯 FrameMaker 시작 - GPS와 FasterLIO 동기화 대기 중...")

    def fasterlio_callback(self, msg):
        """FasterLIO 데이터 수신 및 원점 설정"""
        # 현재 pose 업데이트
        self.current_body_pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z
        }
        
        # 첫 번째 FasterLIO 데이터면 원점으로 설정
        if self.fasterlio_origin is None:
            self.fasterlio_origin = self.current_body_pose
            rospy.loginfo(f"✅ FasterLIO 원점 설정: {self.fasterlio_origin}")

    def gps_callback(self, msg):
        """GPS 데이터 수신 및 UTM Local 원점 설정"""
        # 실시간 GPS 데이터 업데이트 (웹 전송용)
        if msg.status.status >= 0:
            self.current_gps = {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "altitude": msg.altitude
            }
        
        # UTM 원점 설정 (한 번만)
        if not self.first_gps_received and msg.status.status >= 0:
            if self.fasterlio_origin is None:
                rospy.logwarn_throttle(5, "⏳ GPS 수신됨, FasterLIO 원점 대기 중...")
                return

            success = self.setup_utm_origin_from_gps(msg.latitude, msg.longitude)
            if success:
                self.first_gps_received = True
                rospy.loginfo("🎉 UTM Local 좌표계 설정 완료!")

    def setup_utm_origin_from_gps(self, lat, lon):
        """🎯 GPS-FasterLIO 동기화된 UTM Local 원점 설정"""
        try:
            rospy.loginfo(f"🔄 UTM Local 원점 설정 시도:")
            rospy.loginfo(f"   GPS: ({lat:.6f}, {lon:.6f})")
            
            # UTM 변환
            if abs(lat) < 0.01 and abs(lon) < 0.01:
                rospy.loginfo("🎮 시뮬레이션 GPS 감지")
                easting = lat * 111320.0
                northing = lon * 111320.0
                zone_num, zone_letter = 52, 'S'
            else:
                rospy.loginfo("🌍 실제 GPS 좌표 처리")
                easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)
            
            rospy.loginfo(f"   절대 UTM: ({easting:.1f}, {northing:.1f}) Zone:{zone_num}{zone_letter}")
            
            # 🎯 핵심: FasterLIO와 GPS 동기화
            if self.current_body_pose:
                # FasterLIO 원점 기준 현재 상대 위치
                fasterlio_rel_x = self.current_body_pose["x"] - self.fasterlio_origin["x"]
                fasterlio_rel_y = self.current_body_pose["y"] - self.fasterlio_origin["y"]
                
                # UTM Local 원점 = 현재 로봇 위치를 (0,0)으로 설정
                # GPS 현재 위치에서 FasterLIO 상대 위치를 빼서 원점 계산
                local_origin_easting = easting - fasterlio_rel_x
                local_origin_northing = northing - fasterlio_rel_y
                
                rospy.loginfo(f"🔄 GPS-FasterLIO 동기화: FasterLIO상대({fasterlio_rel_x:.2f}, {fasterlio_rel_y:.2f}) → 로봇GPS({easting:.1f}, {northing:.1f}) → UTM원점({local_origin_easting:.1f}, {local_origin_northing:.1f})")
                
                self.utm_origin_absolute = {
                    "easting": local_origin_easting,
                    "northing": local_origin_northing,
                    "lat": lat,
                    "lon": lon
                }
            else:
                # FasterLIO 위치 정보가 없으면 GPS 위치를 원점으로 설정
                rospy.logwarn("⚠️ FasterLIO 현재 위치 없음, GPS 위치를 원점으로 설정")
                self.utm_origin_absolute = {
                    "easting": easting,
                    "northing": northing,
                    "lat": lat,
                    "lon": lon
                }
            
            self.utm_zone = f"{zone_num}{zone_letter}"
            
            # UTM 원점 정보 발행
            self.publish_utm_origin_info()
            
            # Static TF 발행
            self.broadcast_static_tf()
            
            rospy.loginfo(f"✅ UTM Local 원점 설정 완료! Zone:{self.utm_zone} 절대UTM({self.utm_origin_absolute['easting']:.1f}, {self.utm_origin_absolute['northing']:.1f}) → 로봇=Local(0,0)")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"❌ UTM 원점 설정 실패: {e}")
            return False

    def publish_utm_origin_info(self):
        """UTM 원점 정보 발행 (다른 노드들이 구독)"""
        if self.utm_origin_absolute:
            origin_info = {
                "utm_zone": self.utm_zone,
                "utm_origin_absolute": self.utm_origin_absolute,
                "coordinate_system": "utm_local",
                "description": "UTM Local coordinate system - Robot's first GPS position becomes (0,0)",
                "synchronized_with_fasterlio": True
            }
            
            self.utm_origin_pub.publish(json.dumps(origin_info))
            rospy.loginfo("📡 UTM 원점 정보 발행 완료")

    def publish_gps_data(self, _):
        """실시간 GPS 데이터 발행 (웹 인터페이스용)"""
        if self.current_gps:
            self.gps_data_pub.publish(json.dumps(self.current_gps))
            rospy.loginfo_throttle(10, f"📡 실시간 GPS → 웹: ({self.current_gps['latitude']:.6f}, {self.current_gps['longitude']:.6f})")
        elif self.utm_origin_absolute:
            # GPS가 없으면 원점 정보라도 전송
            fallback_gps = {
                "latitude": self.utm_origin_absolute["lat"],
                "longitude": self.utm_origin_absolute["lon"]
            }
            self.gps_data_pub.publish(json.dumps(fallback_gps))
            rospy.loginfo_throttle(20, f"📡 Fallback GPS → 웹: ({fallback_gps['latitude']:.6f}, {fallback_gps['longitude']:.6f})")

    def broadcast_static_tf(self):
        """Static TF 발행: map → utm_local"""
        if self.utm_origin_absolute:
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "utm_local"
            t.transform.translation.x = 0
            t.transform.translation.y = 0
            t.transform.translation.z = 0
            t.transform.rotation.x = 0
            t.transform.rotation.y = 0
            t.transform.rotation.z = 0
            t.transform.rotation.w = 1
            
            self.tf_broadcaster.sendTransform(t)
            rospy.loginfo("📡 Static TF 발행: map → utm_local")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        fm = FrameMaker()
        fm.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 FrameMaker 종료")
    except Exception as e:
        rospy.logerr(f"❌ FrameMaker 오류: {e}")