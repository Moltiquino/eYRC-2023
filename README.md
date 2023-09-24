#Instructions

Choose the workspace

    cd into any ws

You likely already have the rclpy and other  packages installed as part of your ROS 2 system. It’s good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:

    rosdep install -i --from-path src --rosdistro humble -y

Still in the root of your workspace, ros2_ws, build your new package:

    colcon build --symlink-install

Finally source the setup.bash:

    source install/setup.bash