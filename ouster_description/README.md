ouster_description
===================

This package is forked from https://github.com/wilselby/ouster_example, which is itself a fork of
https://github.com/ouster-lidar/ouster_example

In order to reduce reliance on third-party components in deployed robots, I've stripped out everything from
Wil Selby's fork except the Gazebo plugins and description package.  This should allow adding the OS-1 lidar
to be added to Clearpath robots, while still using the manufacturer's ROS driver in an unmodified state.

Example use
------------

To add the lidar to a robot, the easiest way is to clone both this repo and https://github.com/ouster-lidar/ouster_example
into your workspace and build as normal.

Then create an accessories URDF file and point e.g. JACKAL_URDF_EXTRAS to point to it.  For example, this will
add the lidar to a Jackal's front mount:

    <?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="$(find ouster_description)/urdf/OS1-64.urdf.xacro" />
      <xacro:OS1-64 parent="front_mount">
        <origin xyz="0 0 0" rpy="0 0 0" />
      </xacro:OS1-64>
    </robot>
