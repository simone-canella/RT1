import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time


class UIpy(Node):
    def __init__(self):
        # NODE INITIALIZATION
        super().__init__('ui_py')

        # PUBLISHER: tell which turtle is moving
        self.moving_turtle_pub = self.create_publisher(
            String,
            '/moving_turtle',
            10
        )

        # SUBSCRIBER: receive stop signal from distance_node
        self.stop_sub = self.create_subscription(
            String,
            '/stop_movement',
            self.stop_callback,
            10
        )

        # boolean variable for stopping movement early
        self.force_stop = False

        # === MAIN LOOP ===
        while rclpy.ok():
            # ask user which turtle to control
            turtle = input("Which turtle do you want to control (turtle1/turtle2)? ")

            # validate input
            if turtle not in ("turtle1", "turtle2"):
                print("Invalid turtle name! Try again.")
                continue

            # build the cmd_vel topic for the selected turtle (containing the turtle name)
            topic = f"/{turtle}/cmd_vel"

            # create a publisher for this turtle
            self.publisher = self.create_publisher(Twist, topic, 10)

            # initialize the control variables
            linear_velocity = float(input("Insert linear velocity: "))
            angular_velocity = float(input("Insert angular velocity: "))

            # initialize the values of the message
            message = Twist()
            message.linear.x = linear_velocity
            message.angular.z = angular_velocity

            # initialize and publish which turtle is moving (for the distance node)
            moving_msg = String()
            moving_msg.data = turtle
            self.moving_turtle_pub.publish(moving_msg)

            # send message for 1 second
            start_time = time.time()
            self.force_stop = False  # reset before movement

            while not self.force_stop and (time.time() - start_time) < 1.0 and rclpy.ok():
                # publish the velocity of the moving turtle
                self.publisher.publish(message)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.1)  # 10 Hz

            # stop moving after 1 second OR early if force_stop == True
            message.linear.x = 0.0
            message.angular.z = 0.0
            self.publisher.publish(message)

            print("Command finished. Insert new command...\n")

    # callback for setting force_stop to true (set by distance_node)
    def stop_callback(self, msg):
        if msg.data == "stop":
            self.force_stop = True
            self.get_logger().warn("STOP received from distance_node")


def main(args=None):
    rclpy.init(args=args)
    node = UIpy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
