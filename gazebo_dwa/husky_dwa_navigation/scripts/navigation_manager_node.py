#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from actionlib_msgs.msg import GoalStatusArray
from dynamic_reconfigure.msg import Config, StrParameter
from dynamic_reconfigure.srv import Reconfigure
import rosgraph_msgs.msg

class NavigationManager:
    def __init__(self):
        # TF 버퍼 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 파라미터 로드
        self.costmap_topic = rospy.get_param('~local_costmap_topic', '/move_base/local_costmap/costmap')
        self.switch_service = rospy.get_param('~switch_service', '/move_base/set_parameters')
        self.planner_param = rospy.get_param('~planner_type_parameter', '/move_base/base_global_planner')
        self.default_planner = rospy.get_param('~default_planner', 'global_planner/GlobalPlanner')
        # GlobalPlanner 사용
        self.alternate_planner = rospy.get_param('~alternate_planner', 'global_planner/GlobalPlanner')
        self.update_rate = rospy.get_param('~update_rate', 2.0)  # 중간 지점 업데이트 주기 (Hz)
        
        # 목표 식별을 위한 특수 z 좌표 값 설정
        self.INTERMEDIATE_GOAL_Z = 0.5  # 중간 목표 식별용 특수 z 값
        self.ORIGINAL_GOAL_Z = None       # 원본 목표의 z 값 (원래 값 유지)
        
        # 장애물 회피 관련 변수
        self.consecutive_failures = 0
        self.plan_failure_count = 0
        self.last_failure_time = rospy.Time(0)
        self.last_plan_failure_time = rospy.Time(0)  # 추가: last_plan_failure_time 초기화
        self.failure_shift_direction = 1  # 1: 동쪽(오른쪽), -1: 서쪽(왼쪽)
        self.failure_shift_distance = 1.0  # 기본 이동 거리 (미터)

        # 추가: 최대 시도 횟수 제한
        self.max_shift_attempts = 6  # 최대 3번의 왼쪽-오른쪽 시도 (총 6번)
        self.current_shift_attempts = 0
        self.increasing_shift_distance = True  # 거리를 점진적으로 증가시킬지 결정
        self.base_shift_distance = 1.0  # 기본 이동 거리
        self.shift_multiplier = 1.0     # 거리 증가를 위한 곱셈 인자
        
        # 상태 변수
        self.costmap = None
        self.current_goal = None
        self.original_goal = None
        self.using_default_planner = True
        self.have_costmap = False
        self.intermediate_goal_active = False
        self.move_base_status = 0  # 0: PENDING, 1: ACTIVE, 2: PREEMPTED, 3: SUCCEEDED, 4: ABORTED ...
        
        # 구독자 설정
        self.costmap_sub = rospy.Subscriber(self.costmap_topic, OccupancyGrid, self.costmap_callback)
        
        # 로그 메시지 구독
        self.log_sub = rospy.Subscriber("/rosout", rosgraph_msgs.msg.Log, self.log_callback)
    
        # 목표 지점 구독
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.current_goal_sub = rospy.Subscriber('/move_base/current_goal', PoseStamped, self.current_goal_callback)
        
        # 중간 목표 지점 발행자
        self.intermediate_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        # 중간 목표 업데이트 타이머
        self.update_timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.update_intermediate_goal)
        
        rospy.loginfo("플래너 전환기 초기화 완료. 목표 지점이 costmap 외부에 있을 경우 주기적으로 중간 목표를 업데이트합니다.")
    
    def log_callback(self, log_msg):
        """
        ROS 로그 메시지를 모니터링하여 'Failed to get a plan.' 에러를 감지
        수정 버전: 자신의 로그 메시지를 재감지하는 문제 해결
        """
        # 1. 확인: 자신이 출력한 로그 메시지는 무시 (로그 중첩 방지)
        if "플랜 실패 감지" in log_msg.msg or "장애물 회피" in log_msg.msg:
            return
        
        # 2. 확인: 메시지 출처가 move_base인지 확인 (플래너 관련 메시지만 처리)
        if "/move_base" not in log_msg.name:
            return
        
        # 3. 확인: 정확한 오류 문자열만 감지
        if "Failed to get a plan." in log_msg.msg:
            current_time = rospy.Time.now()
        
            # 최근 처리한 실패와의 시간 간격 확인 (짧은 시간 내 중복 처리 방지)
            if (current_time - self.last_plan_failure_time).to_sec() < 1.0:  # 1초 이내 중복 방지
                return
            
            # 기존 카운터 로직
            if (current_time - self.last_plan_failure_time).to_sec() > 5.0:
                self.plan_failure_count = 0
        
            self.plan_failure_count += 1
            self.last_plan_failure_time = current_time
        
            rospy.logwarn("플랜 실패 감지: %d번째 (원본 오류: Failed to get a plan.)", 
                         self.plan_failure_count)
        
            # 추가: 로그 디버깅을 위한 정보 출력
            rospy.logdebug("로그 출처: %s, 레벨: %d, 시간: %.2f", 
                          log_msg.name, log_msg.level, log_msg.header.stamp.to_sec())
        
            # 2번 연속 실패하면 shift_intermediate_goal 실행
            if self.plan_failure_count >= 2:
                rospy.logwarn("연속 %d번 경로 생성 실패: 중간 목표 위치를 이동합니다", 
                             self.plan_failure_count)
            
                # 중요: 실행 전 카운터 리셋 (중복 실행 방지)
                self.plan_failure_count = 0
            
                # 시도 사이에 약간의 지연 추가 (시스템이 안정화될 시간 제공)
                rospy.sleep(0.5)
            
                # 경로 실패 처리 함수 호출
                self.shift_intermediate_goal()
    
    def is_intermediate_goal(self, goal_msg):
        """
        주어진 목표가 중간 목표인지 확인하는 함수
        z 좌표 값으로 구분
        """
        # z 좌표가 중간 목표용 특수 값인지 확인
        return abs(goal_msg.pose.position.z - self.INTERMEDIATE_GOAL_Z) < 0.1
    
    def update_intermediate_goal(self, event=None):
        """
        중간 목표를 주기적으로 업데이트하는 타이머 콜백
        """
        # 원본 목표가 없거나 costmap을 아직 받지 못했다면 무시
        if self.original_goal is None or not self.have_costmap:
            return
            
        # 현재 로봇의 위치를 가져오기
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return
            
        # 원본 목표 위치를 costmap 프레임으로 변환
        try:
            if self.original_goal.header.frame_id != self.costmap.header.frame_id:
                goal_in_costmap_frame = self.tf_buffer.transform(self.original_goal, self.costmap.header.frame_id, rospy.Duration(1.0))
            else:
                goal_in_costmap_frame = self.original_goal
                
            goal_x = goal_in_costmap_frame.pose.position.x
            goal_y = goal_in_costmap_frame.pose.position.y
            
            # costmap 경계 확인
            map_origin_x = self.costmap.info.origin.position.x
            map_origin_y = self.costmap.info.origin.position.y
            map_width = self.costmap.info.width * self.costmap.info.resolution
            map_height = self.costmap.info.height * self.costmap.info.resolution
            
            # 목표 지점이 costmap 경계 밖에 있는지 확인
            is_outside = (goal_x < map_origin_x or 
                          goal_y < map_origin_y or
                          goal_x > map_origin_x + map_width or
                          goal_y > map_origin_y + map_height)
            
            # 목표가 costmap 내부에 있는 경우에만 원본 목표로 전환
            if not is_outside:
                if self.intermediate_goal_active:
                    rospy.loginfo("목표가 costmap 내부에 있습니다. 원본 목표로 전환합니다.")
                    self.switch_to_original_goal()
                return
            
            # 중간 목표가 활성화되어 있고, 마지막 업데이트 후 충분한 시간이 지나지 않았다면 무시
            if self.intermediate_goal_active and hasattr(self, 'last_intermediate_goal_time') and \
               (rospy.Time.now() - self.last_intermediate_goal_time).to_sec() < 3.0:  # 3초마다 업데이트
                return
            
            # 로봇에서 목표까지의 직선과 costmap 경계의 교차점 계산
            intersection_point = self.find_intersection_with_boundary(
                robot_pose.x, robot_pose.y, 
                goal_x, goal_y,
                map_origin_x, map_origin_y, 
                map_origin_x + map_width, map_origin_y + map_height
            )
            
            if intersection_point:
                # 이전 중간 목표와 새로운 중간 목표 사이의 거리가 충분히 크면 업데이트
                update_needed = True
                if self.intermediate_goal_active and self.current_goal is not None:
                    current_intermediate_x = self.current_goal.pose.position.x
                    current_intermediate_y = self.current_goal.pose.position.y
                    dist_to_new_intermediate = np.sqrt((intersection_point[0] - current_intermediate_x)**2 + 
                                                       (intersection_point[1] - current_intermediate_y)**2)
                    update_needed = dist_to_new_intermediate > 1.0  # 1미터 이상 차이나면 업데이트
                
                if update_needed or hasattr(self, 'last_goal_reached'):
                    # 새로운 중간 목표 생성 및 발행
                    intermediate_goal = PoseStamped()
                    intermediate_goal.header.frame_id = self.costmap.header.frame_id
                    intermediate_goal.header.stamp = rospy.Time.now()
                    intermediate_goal.pose.position.x = intersection_point[0]
                    intermediate_goal.pose.position.y = intersection_point[1]
                    
                    # 중간 목표 식별을 위한 특수 z 값 설정
                    intermediate_goal.pose.position.z = self.INTERMEDIATE_GOAL_Z
                    
                    # 방향은 원래 목표를 향하도록 설정
                    direction = np.arctan2(goal_y - intersection_point[1], 
                                          goal_x - intersection_point[0])
                    
                    # 쿼터니언으로 변환 (z축 회전만 고려)
                    intermediate_goal.pose.orientation.z = np.sin(direction / 2)
                    intermediate_goal.pose.orientation.w = np.cos(direction / 2)
                    
                    # GlobalPlanner 사용 확인
                    if not self.using_default_planner:
                        self.switch_planner(self.default_planner)
                        self.using_default_planner = True
                    
                    # 중간 목표 발행
                    self.intermediate_goal_pub.publish(intermediate_goal)
                    self.last_intermediate_goal_time = rospy.Time.now()
                    self.intermediate_goal_active = True
                    if hasattr(self, 'last_goal_reached'):
                        delattr(self, 'last_goal_reached')
                    
                    # 거리 로깅 추가
                    robot_to_intermediate_dist = np.sqrt((robot_pose.x - intersection_point[0])**2 + 
                                                         (robot_pose.y - intersection_point[1])**2)
                    intermediate_to_goal_dist = np.sqrt((intersection_point[0] - goal_x)**2 + 
                                                        (intersection_point[1] - goal_y)**2)
                    
                    rospy.loginfo("새 중간 목표 설정: (%.2f, %.2f) - 로봇으로부터 %.2fm, 목표까지 %.2fm", 
                                 intersection_point[0], intersection_point[1],
                                 robot_to_intermediate_dist, intermediate_to_goal_dist)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("목표 지점 변환 중 오류: %s", str(e))

    def reset_and_try_further_goal(self):
        """
        여러 번의 이동 시도가 실패한 후 다른 접근 방식 시도
        """
        # 원본 목표와 로봇 사이의 방향 계산
        robot_pose = self.get_robot_pose()
        if robot_pose is None or self.original_goal is None:
            return
    
        try:
            # 원본 목표를 costmap 프레임으로 변환
            if self.original_goal.header.frame_id != self.costmap.header.frame_id:
                goal_in_costmap_frame = self.tf_buffer.transform(self.original_goal, self.costmap.header.frame_id, rospy.Duration(1.0))
            else:
                goal_in_costmap_frame = self.original_goal
        
            goal_x = goal_in_costmap_frame.pose.position.x
            goal_y = goal_in_costmap_frame.pose.position.y
        
            # 로봇에서 목표까지의 방향 벡터
            dx = goal_x - robot_pose.x
            dy = goal_y - robot_pose.y
        
            # 벡터 정규화
            distance = np.sqrt(dx*dx + dy*dy)
            if distance > 0:
                dx /= distance
                dy /= distance
        
            # costmap 크기 가져오기
            map_width = self.costmap.info.width * self.costmap.info.resolution
            map_height = self.costmap.info.height * self.costmap.info.resolution
        
            # 더 가까운 중간 목표 생성 (로봇 위치에서 반대방향으로)
            new_x = robot_pose.x - dx * map_width * 0.4  # costmap 폭의 40% 거리로 뒤로 이동
            new_y = robot_pose.y - dy * map_height * 0.4  # costmap 높이의 40% 거리로 뒤로 이동
        
            # 중간 목표 생성
            intermediate_goal = PoseStamped()
            intermediate_goal.header.frame_id = self.costmap.header.frame_id
            intermediate_goal.header.stamp = rospy.Time.now()
            intermediate_goal.pose.position.x = new_x
            intermediate_goal.pose.position.y = new_y
            intermediate_goal.pose.position.z = self.INTERMEDIATE_GOAL_Z
        
            # 방향은 원래 목표를 향하도록
            direction = np.arctan2(goal_y - new_y, goal_x - new_x)
            intermediate_goal.pose.orientation.z = np.sin(direction / 2)
            intermediate_goal.pose.orientation.w = np.cos(direction / 2)
        
            # 시도 횟수 및 관련 변수 초기화
            self.current_shift_attempts = 0
            self.shift_multiplier = 1.0
        
            # 새 중간 목표 발행
            self.intermediate_goal_pub.publish(intermediate_goal)
            self.last_intermediate_goal_time = rospy.Time.now()
        
            rospy.loginfo("새로운 접근 방법: 로봇 근처에 더 가까운 중간 목표 설정 (%.2f, %.2f)", new_x, new_y)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("목표 지점 변환 중 오류: %s", str(e))


    def shift_intermediate_goal(self):
        """
        장애물로 인해 경로 생성이 실패할 때 중간 목표 지점을 측면으로 이동시킨다.
        """
        if not self.have_costmap or self.current_goal is None:
            return
        
        # 최대 시도 횟수 확인
        if self.current_shift_attempts >= self.max_shift_attempts:
            rospy.logwarn("최대 %d번 중간 목표 이동 시도했으나 실패. 더 멀리 떨어진 중간 목표를 시도합니다.", self.max_shift_attempts)
            self.reset_and_try_further_goal()
            return
    
        # 현재 로봇 위치 가져오기
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return
    
        try:
            # 현재 중간 목표 위치
            current_intermediate_x = self.current_goal.pose.position.x
            current_intermediate_y = self.current_goal.pose.position.y
        
            # 로봇에서 현재 중간 목표까지의 방향 벡터
            intermediate_direction_x = current_intermediate_x - robot_pose.x
            intermediate_direction_y = current_intermediate_y - robot_pose.y
        
            # 벡터 정규화
            direction_length = np.sqrt(intermediate_direction_x**2 + intermediate_direction_y**2)
            if direction_length > 0:
                intermediate_direction_x /= direction_length
                intermediate_direction_y /= direction_length
        
            # 현재 진행 방향에 수직인 벡터 계산
            perpendicular_x = -intermediate_direction_y  # 오른쪽 방향
            perpendicular_y = intermediate_direction_x   # 위쪽 방향
        
            # 이동 거리 계산 - 점진적으로 증가
            if self.increasing_shift_distance and self.current_shift_attempts % 2 == 0:
                self.shift_multiplier *= 1.5  # 방향 전환 시마다 1.5배 증가
        
            shift_distance = self.base_shift_distance * self.shift_multiplier
        
            # 새 중간 목표 위치 계산
            new_intermediate_x = current_intermediate_x + perpendicular_x * shift_distance * self.failure_shift_direction
            new_intermediate_y = current_intermediate_y + perpendicular_y * shift_distance * self.failure_shift_direction
        
            # 새 중간 목표 생성
            intermediate_goal = PoseStamped()
            intermediate_goal.header.frame_id = self.costmap.header.frame_id
            intermediate_goal.header.stamp = rospy.Time.now()
            intermediate_goal.pose.position.x = new_intermediate_x
            intermediate_goal.pose.position.y = new_intermediate_y
            
            # 중간 목표 식별을 위한 특수 z 값 설정
            intermediate_goal.pose.position.z = self.INTERMEDIATE_GOAL_Z
            
            # 방향은 기존 중간 목표와 동일하게 유지
            intermediate_goal.pose.orientation = self.current_goal.pose.orientation
        
            # GlobalPlanner 사용 확인 
            if not self.using_default_planner:
                self.switch_planner(self.default_planner)
                self.using_default_planner = True
        
            # 중간 목표 발행
            self.intermediate_goal_pub.publish(intermediate_goal)
            self.last_intermediate_goal_time = rospy.Time.now()

            # 시도 횟수 증가
            self.current_shift_attempts += 1
        
            # 다음 번 실패 시 반대 방향으로 시도하기 위해 방향 전환
            self.failure_shift_direction *= -1
        
            rospy.loginfo("장애물 회피를 위해 중간 목표 이동 (%d/%d): (%.2f, %.2f) -> (%.2f, %.2f), 이동거리: %.2fm", 
                     self.current_shift_attempts, self.max_shift_attempts,
                     current_intermediate_x, current_intermediate_y,
                     new_intermediate_x, new_intermediate_y,
                     shift_distance)
    
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("목표 지점 변환 중 오류: %s", str(e))


    def switch_to_original_goal(self):
        """
        원래 목표로 전환하는 함수
        """
        if self.original_goal is None:
            rospy.logwarn("원래 목표가 없습니다.")
            return
            
        rospy.loginfo("원래 목표로 전환합니다: (%.2f, %.2f)", 
                     self.original_goal.pose.position.x, 
                     self.original_goal.pose.position.y)
                     
        # GlobalPlanner로 전환 
        self.switch_planner(self.alternate_planner)
        self.using_default_planner = False
        self.intermediate_goal_active = False
        
        # 원래 목표의 타임스탬프 업데이트
        original_goal_updated = PoseStamped()
        original_goal_updated.header = self.original_goal.header
        original_goal_updated.header.stamp = rospy.Time.now()  # 현재 시간으로 업데이트
        original_goal_updated.pose = self.original_goal.pose
        
        # z 좌표가 변경되었을 수 있으므로 원래 목표의 z 값 복원
        if self.ORIGINAL_GOAL_Z is not None:
            original_goal_updated.pose.position.z = self.ORIGINAL_GOAL_Z
        
        # 약간의 지연 후 원래 목표 발행 (move_base가 이전 목표를 처리할 시간을 주기 위함)
        rospy.sleep(0.5)
        self.intermediate_goal_pub.publish(original_goal_updated)
        rospy.loginfo("원래 목표를 발행했습니다.")
    
    def costmap_callback(self, costmap_msg):
        """
        Costmap 메시지를 받아서 저장하는 콜백
        """
        self.costmap = costmap_msg
        self.have_costmap = True
    
    def goal_callback(self, goal_msg):
        """
        새 목표 지점이 설정되었을 때의 콜백
        """
        # 만약 이 목표가 중간 목표라면 무시 (z 좌표로 식별)
        if self.is_intermediate_goal(goal_msg):
            rospy.logdebug("중간 목표를 받았습니다. 원본 목표 설정을 건너뜁니다.")
            return
            
        # 원본 목표 저장
        self.original_goal = goal_msg
        # 원본 목표의 z 좌표 저장
        self.ORIGINAL_GOAL_Z = goal_msg.pose.position.z
        self.intermediate_goal_active = False
        self.consecutive_failures = 0  # 새 목표 설정시 실패 카운터 초기화
        
        rospy.loginfo("새 원본 목표 설정: (%.2f, %.2f)", 
                    goal_msg.pose.position.x, goal_msg.pose.position.y)
        
        # 중간 목표 업데이트 트리거
        self.update_intermediate_goal()
        self.current_shift_attempts = 0
        self.shift_multiplier = 1.0
    
    def current_goal_callback(self, goal_msg):
        """
        현재 목표 지점 업데이트 콜백
        """
        self.current_goal = goal_msg
    
    def get_robot_pose(self):
        """
        로봇의 현재 위치를 가져옴
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                self.costmap.header.frame_id, 'base_link', rospy.Time(0), rospy.Duration(1.0)
            )
            return trans.transform.translation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"로봇 위치 확인 중 오류: {str(e)}")
            return None
    
    def find_intersection_with_boundary(self, x1, y1, x2, y2, min_x, min_y, max_x, max_y):
        """
        직선(x1,y1 -> x2,y2)과 직사각형 경계의 교차점을 계산
        """
        # 직선의 방정식: y = mx + b
        if x2 - x1 == 0:  # 수직선
            if min_x <= x1 <= max_x:
                # y 범위 내에서 교차점 찾기
                if y1 < y2:  # 위로 향하는 경우
                    return [x1, max_y] if y1 < max_y < y2 else None
                else:  # 아래로 향하는 경우
                    return [x1, min_y] if y2 < min_y < y1 else None
            return None
        
        m = (y2 - y1) / (x2 - x1)  # 기울기
        b = y1 - m * x1  # y절편
        
        # 직사각형의 네 변과의 교차점 계산
        intersections = []
        
        # 왼쪽 변: x = min_x
        y_at_left = m * min_x + b
        if min_y <= y_at_left <= max_y:
            intersections.append((min_x, y_at_left))
        
        # 오른쪽 변: x = max_x
        y_at_right = m * max_x + b
        if min_y <= y_at_right <= max_y:
            intersections.append((max_x, y_at_right))
        
        # 아래쪽 변: y = min_y
        x_at_bottom = (min_y - b) / m if m != 0 else None
        if x_at_bottom is not None and min_x <= x_at_bottom <= max_x:
            intersections.append((x_at_bottom, min_y))
        
        # 위쪽 변: y = max_y
        x_at_top = (max_y - b) / m if m != 0 else None
        if x_at_top is not None and min_x <= x_at_top <= max_x:
            intersections.append((x_at_top, max_y))
        
        if not intersections:
            return None
        
        # 로봇에서 목표를 향하는 방향에 있는 첫 번째 교차점 찾기
        robot_to_goal_vector = np.array([x2 - x1, y2 - y1])
        robot_to_goal_dist = np.linalg.norm(robot_to_goal_vector)
        
        closest_intersection = None
        closest_dist = float('inf')
        
        for ix, iy in intersections:
            robot_to_intersection_vector = np.array([ix - x1, iy - y1])
            # 같은 방향인지 확인 (내적이 양수)
            dot_product = np.dot(robot_to_goal_vector, robot_to_intersection_vector)
            
            if dot_product > 0:  # 같은 방향
                dist = np.linalg.norm(robot_to_intersection_vector)
                if dist < closest_dist and dist < robot_to_goal_dist:
                    closest_dist = dist
                    closest_intersection = (ix, iy)
        
        return closest_intersection
    
    def switch_planner(self, planner_type):
        """
        dynamic_reconfigure를 사용하여 플래너 유형 변경
        """
        try:
            # dynamic_reconfigure 서비스 클라이언트 설정
            client = rospy.ServiceProxy(self.switch_service, Reconfigure)
            
            # 설정 메시지 생성
            config = Config()
            string_param = StrParameter()
            string_param.name = "base_global_planner"
            string_param.value = planner_type
            config.strs = [string_param]
            
            # 서비스 호출
            response = client(config)
            rospy.loginfo("플래너 전환 완료: %s", planner_type)
            
        except rospy.ServiceException as e:
            rospy.logerr("서비스 호출 실패: %s", str(e))

def main():
    rospy.init_node('navigation_manager_node')
    
    manager = NavigationManager()
    
    rospy.loginfo("플래너 전환 노드가 실행 중입니다...")
    rospy.spin()

if __name__ == '__main__':
    main()