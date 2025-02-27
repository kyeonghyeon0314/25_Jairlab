# Simplified Pseudocode for Local Planner
def local_planner():
    while True:
        # Sensor inputs
        global_path = get_kakao_gps_waypoints()
        current_pose = fasterlio.get_odometry()
        obstacles = frnet_lasermix.detect_objects()
        
        # Fusion and prediction
        fused_pose = ekf_fusion(gps=global_path, lidar_odom=current_pose)
        dynamic_obstacles = predict_trajectories(obstacles)
        
        # Path generation
        local_waypoints = tangent_bug(
            start=fused_pose, 
            goal=global_path.next_waypoint(),
            obstacles=dynamic_obstacles
        )
        
        # Safety check and execution
        if is_safe(local_waypoints):
            husky.execute(local_waypoints)
        else:
            enter_recovery_mode()
