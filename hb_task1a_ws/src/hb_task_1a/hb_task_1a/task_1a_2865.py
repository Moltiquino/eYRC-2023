"""
# Team ID:          < Team-ID >
# Theme:            < Theme Name >
# Author List:      < Names of team members worked on this file separated by Comma >
# Filename:         < Filename >
# Functions:        < Comma separated list of functions in this file >
# Global variables: < List of global variables defined in this file, None if no global variables >
"""


from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import rclpy
from math import sqrt


class DrawCircleBig(Node):
    """
    Purpose:
    ---
    < Short-text describing the purpose of this function >

    Input Arguments:
    ---
    `< name of 1st input argument >` :  [ < type of 1st input argument > ]
        < one-line description of 1st input argument >

    `< name of 2nd input argument >` :  [ < type of 2nd input argument > ]
        < one-line description of 2nd input argument >

    Returns:
    ---
    `< name of 1st return argument >` :  [ < type of 1st return argument > ]
        < one-line description of 1st return argument >

    `< name of 2nd return argument >` :  [ < type of 2nd return argument > ]
        < one-line description of 2nd return argument >

    Example call:
    ---
    < Example of how to call this function >
    """

    def __init__(self):
        """
        Purpose:
        ---
        < Short-text describing the purpose of this function >

        Input Arguments:
        ---
        `< name of 1st input argument >` :  [ < type of 1st input argument > ]
            < one-line description of 1st input argument >

        `< name of 2nd input argument >` :  [ < type of 2nd input argument > ]
            < one-line description of 2nd input argument >

        Returns:
        ---
        `< name of 1st return argument >` :  [ < type of 1st return argument > ]
            < one-line description of 1st return argument >

        `< name of 2nd return argument >` :  [ < type of 2nd return argument > ]
            < one-line description of 2nd return argument >

        Example call:
        ---
        < Example of how to call this function >
        """

        super().__init__("draw_circle_big")
        self.pub_ = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)
        self.sub_ = self.create_subscription(
            Pose, "/turtle2/pose", self.pose_callbck, 10
        )
        self.timer = self.create_timer(0.7, self.move_)
        self.pose = None
        self.init_pose = None
        self.done = False
        self.dist = 0
        self.r = 2.0
        self.tol = 0.1

    def euclidean_distance(self, goal_pose):
        """
        Purpose:
        ---
        < Short-text describing the purpose of this function >

        Input Arguments:
        ---
        `< name of 1st input argument >` :  [ < type of 1st input argument > ]
            < one-line description of 1st input argument >

        `< name of 2nd input argument >` :  [ < type of 2nd input argument > ]
            < one-line description of 2nd input argument >

        Returns:
        ---
        `< name of 1st return argument >` :  [ < type of 1st return argument > ]
            < one-line description of 1st return argument >

        `< name of 2nd return argument >` :  [ < type of 2nd return argument > ]
            < one-line description of 2nd return argument >

        Example call:
        ---
        < Example of how to call this function >
        """

        return sqrt(
            pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)
        )

    def pose_callbck(self, pose_):
        """
        Purpose:
        ---
        < Short-text describing the purpose of this function >

        Input Arguments:
        ---
        `< name of 1st input argument >` :  [ < type of 1st input argument > ]
            < one-line description of 1st input argument >

        `< name of 2nd input argument >` :  [ < type of 2nd input argument > ]
            < one-line description of 2nd input argument >

        Returns:
        ---
        `< name of 1st return argument >` :  [ < type of 1st return argument > ]
            < one-line description of 1st return argument >

        `< name of 2nd return argument >` :  [ < type of 2nd return argument > ]
            < one-line description of 2nd return argument >

        Example call:
        ---
        < Example of how to call this function >
        """
        self.pose = pose_
        if not self.init_pose:
            self.init_pose = pose_

    def move_(self):
        """
        Purpose:
        ---
        < Short-text describing the purpose of this function >

        Input Arguments:
        ---
        `< name of 1st input argument >` :  [ < type of 1st input argument > ]
            < one-line description of 1st input argument >

        `< name of 2nd input argument >` :  [ < type of 2nd input argument > ]
            < one-line description of 2nd input argument >

        Returns:
        ---
        `< name of 1st return argument >` :  [ < type of 1st return argument > ]
            < one-line description of 1st return argument >

        `< name of 2nd return argument >` :  [ < type of 2nd return argument > ]
            < one-line description of 2nd return argument >

        Example call:
        ---
        < Example of how to call this function >
        """
        omega = 1.0
        if self.init_pose == None:
            self.get_logger().warn("Turtle sim not active")
            return

        msg = Twist()
        if (self.euclidean_distance(self.init_pose) < self.tol) and (self.dist > 5.0):
            self.done = True
            omega = 0.0

        msg.linear.x = omega * self.r
        msg.angular.z = -1.0 * omega
        self.dist += omega
        self.get_logger().info(str(self.dist))
        self.pub_.publish(msg)


class DrawCircle(Node):
    """
    Purpose:
    ---
    < Short-text describing the purpose of this function >

    Input Arguments:
    ---
    `< name of 1st input argument >` :  [ < type of 1st input argument > ]
        < one-line description of 1st input argument >

    `< name of 2nd input argument >` :  [ < type of 2nd input argument > ]
        < one-line description of 2nd input argument >

    Returns:
    ---
    `< name of 1st return argument >` :  [ < type of 1st return argument > ]
        < one-line description of 1st return argument >

    `< name of 2nd return argument >` :  [ < type of 2nd return argument > ]
        < one-line description of 2nd return argument >

    Example call:
    ---
    < Example of how to call this function >
    """

    def __init__(self):
        """
        Purpose:
        ---
        < Short-text describing the purpose of this function >

        Input Arguments:
        ---
        `< name of 1st input argument >` :  [ < type of 1st input argument > ]
            < one-line description of 1st input argument >

        `< name of 2nd input argument >` :  [ < type of 2nd input argument > ]
            < one-line description of 2nd input argument >

        Returns:
        ---
        `< name of 1st return argument >` :  [ < type of 1st return argument > ]
            < one-line description of 1st return argument >

        `< name of 2nd return argument >` :  [ < type of 2nd return argument > ]
            < one-line description of 2nd return argument >

        Example call:
        ---
        < Example of how to call this function >
        """
        super().__init__("draw_cricle")
        self.pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.sub_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callbck, 10
        )
        self.timer = self.create_timer(0.7, self.move_)
        self.pose = None
        self.init_pose = None
        self.done = False
        self.dist = 0
        self.r = 1.2
        self.tol = 0.1

    def euclidean_distance(self, goal_pose):
        """
        Purpose:
        ---
        < Short-text describing the purpose of this function >

        Input Arguments:
        ---
        `< name of 1st input argument >` :  [ < type of 1st input argument > ]
            < one-line description of 1st input argument >

        `< name of 2nd input argument >` :  [ < type of 2nd input argument > ]
            < one-line description of 2nd input argument >

        Returns:
        ---
        `< name of 1st return argument >` :  [ < type of 1st return argument > ]
            < one-line description of 1st return argument >

        `< name of 2nd return argument >` :  [ < type of 2nd return argument > ]
            < one-line description of 2nd return argument >

        Example call:
        ---
        < Example of how to call this function >
        """
        return sqrt(
            pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)
        )

    def req_spawn(self):
        """
        Purpose:
        ---
        < Short-text describing the purpose of this function >

        Input Arguments:
        ---
        `< name of 1st input argument >` :  [ < type of 1st input argument > ]
            < one-line description of 1st input argument >

        `< name of 2nd input argument >` :  [ < type of 2nd input argument > ]
            < one-line description of 2nd input argument >

        Returns:
        ---
        `< name of 1st return argument >` :  [ < type of 1st return argument > ]
            < one-line description of 1st return argument >

        `< name of 2nd return argument >` :  [ < type of 2nd return argument > ]
            < one-line description of 2nd return argument >

        Example call:
        ---
        < Example of how to call this function >
        """
        clt = self.create_client(Spawn, "/spawn")
        while not clt.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        my_req = Spawn.Request()

        my_req.x = self.init_pose.x
        my_req.y = self.init_pose.y
        my_req.theta = self.init_pose.theta
        future = clt.call_async(my_req)

        min_pub2 = DrawCircleBig()

        while min_pub2.done == False:
            rclpy.spin_once(min_pub2)

        min_pub2.destroy_node()

    def move_(self):
        """
        Purpose:
        ---
        < Short-text describing the purpose of this function >

        Input Arguments:
        ---
        `< name of 1st input argument >` :  [ < type of 1st input argument > ]
            < one-line description of 1st input argument >

        `< name of 2nd input argument >` :  [ < type of 2nd input argument > ]
            < one-line description of 2nd input argument >

        Returns:
        ---
        `< name of 1st return argument >` :  [ < type of 1st return argument > ]
            < one-line description of 1st return argument >

        `< name of 2nd return argument >` :  [ < type of 2nd return argument > ]
            < one-line description of 2nd return argument >

        Example call:
        ---
        < Example of how to call this function >
        """
        omega = 1.0
        if self.init_pose == None:
            self.get_logger().warn("Turtle sim not active")
            return
        msg = Twist()

        if (self.euclidean_distance(self.init_pose) < self.tol) and (self.dist > 5.0):
            self.req_spawn()
            self.done = True
            omega = 0.0

        msg.linear.x = omega * self.r
        msg.angular.z = omega
        self.dist += omega
        self.get_logger().info(str(self.dist))
        self.pub_.publish(msg)

    def pose_callbck(self, pose_):
        self.pose = pose_
        if not self.init_pose:
            self.init_pose = pose_


def main(args=None):
    """
    Purpose:
    ---
    < Short-text describing the purpose of this function >

    Input Arguments:
    ---
    `< name of 1st input argument >` :  [ < type of 1st input argument > ]
        < one-line description of 1st input argument >

    `< name of 2nd input argument >` :  [ < type of 2nd input argument > ]
        < one-line description of 2nd input argument >

    Returns:
    ---
    `< name of 1st return argument >` :  [ < type of 1st return argument > ]
        < one-line description of 1st return argument >

    `< name of 2nd return argument >` :  [ < type of 2nd return argument > ]
        < one-line description of 2nd return argument >

    Example call:
    ---
    < Example of how to call this function >
    """
    rclpy.init(args=args)
    min_pub = DrawCircle()
    while min_pub.done == False:
        rclpy.spin_once(min_pub)
    min_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
