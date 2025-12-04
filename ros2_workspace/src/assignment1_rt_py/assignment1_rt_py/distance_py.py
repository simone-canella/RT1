import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

import math


class DistanceNodePy(Node):
    def __init__(self):
        # === NODE NAME ===
        super().__init__('distance_node_py')

        # === SUBSCRIBERS ===
        # subscriber to turtle1 pose
        self.t1_pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.t1_pose_callback,
            10
        )

        # subscriber to turtle2 pose
        self.t2_pose_sub = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.t2_pose_callback,
            10
        )

        # subscriber to topic which tells us which turtle is moving
        self.moving_turtle_sub = self.create_subscription(
            String,
            '/moving_turtle',
            self.moving_turtle_callback,
            10
        )

        # === PUBLISHERS ===
        # publisher for the distance between turtles
        self.distance_pub = self.create_publisher(Float32, '/turtles_distance', 10)

        # publishers to stop turtles
        self.turtle1_stop_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.turtle2_stop_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # publisher for sending stop signal to UI
        self.stop_pub = self.create_publisher(String, '/stop_movement', 10)

        # === INTERNAL STATE VARIABLES ===
        # pose variables
        self.t1_x = 0.0
        self.t1_y = 0.0
        self.t2_x = 0.0
        self.t2_y = 0.0

        # name of the turtle currently moving ("turtle1" or "turtle2")
        self.moving_turtle = ""

        # thresholds
        self.distance_threshold = 2.0
        self.min_boundary = 1.0
        self.max_boundary = 10.0

        # === TIMER ===
        # timer for periodic distance computation and safety checks (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Distance node (Python version) started.")


    # === CALLBACKS ===
    # callback for turtle1 pose
    def t1_pose_callback(self, msg):
        self.t1_x = msg.x
        self.t1_y = msg.y

    # callback for turtle2 pose
    def t2_pose_callback(self, msg):
        self.t2_x = msg.x
        self.t2_y = msg.y

    # callback for moving turtle topic
    def moving_turtle_callback(self, msg):
        self.moving_turtle = msg.data
        self.get_logger().info(f"Currently moving turtle: {self.moving_turtle}")

    # === TIMER CALLBACK: live distance computation + safety logic ===
    def timer_callback(self):
        # compute distance between the two turtles
        distance = math.sqrt((self.t1_x - self.t2_x)**2 + (self.t1_y - self.t2_y)**2)

        # initialize and publish the distance
        dist_msg = Float32()
        dist_msg.data = distance
        self.distance_pub.publish(dist_msg)

        # STOP conditions (distance + boundaries)
        if (
            self.moving_turtle == "turtle1" and
            (
                self.t1_x < self.min_boundary or
                self.t1_x > self.max_boundary or
                self.t1_y < self.min_boundary or
                self.t1_y > self.max_boundary or
                distance < self.distance_threshold
            )
        ):
            self.stop_turtle("turtle1")

        elif (
            self.moving_turtle == "turtle2" and
            (
                self.t2_x < self.min_boundary or
                self.t2_x > self.max_boundary or
                self.t2_y < self.min_boundary or
                self.t2_y > self.max_boundary or
                distance < self.distance_threshold
            )
        ):
            self.stop_turtle("turtle2")


    # === INTERNAL HELPER FUNCTION ===
    def stop_turtle(self, name):
        # stop the moving turtle
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0

        if name == "turtle1":
            self.turtle1_stop_pub.publish(stop_msg)
        else:
            self.turtle2_stop_pub.publish(stop_msg)

        # send stop signal to UI
        ui_stop = String()
        ui_stop.data = "stop"
        self.stop_pub.publish(ui_stop)

        self.get_logger().warn(
            f"TURTLE STOP: {name} is stopped because too close to another turtle or to the boundaries"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DistanceNodePy()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
