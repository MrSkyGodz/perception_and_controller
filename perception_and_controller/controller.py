import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
import numpy as np
from tf_transformations import euler_from_quaternion
import time

class Controller(Node):
    def __init__(self):
        super().__init__("controller_node")

        self.wheelbase = 2.0
        self.linear_velocity = 2.0 # m/s
        self.goal : PoseStamped = None
        self.current_pose : Pose = None
        goal_sub= self.create_subscription(PoseStamped,"pose_msg",self.goal_cb,10)
        curr_pose_cub = self.create_subscription(PoseArray,"pose_info",self.pose_cb,10)
        self.cmd_vel_publisher = self.create_publisher(Twist,"/cmd_vel",10)

        self.create_timer(0.2, self.timer_cb)


    def goal_cb(self, msg):
        self.goal = msg

    def pose_cb(self,msg:PoseArray):

        self.current_pose = msg.poses[1]

    def timer_cb(self):
        if self.goal is None or self.current_pose is None:
            self.get_logger().info("Goal or current pose is not received.")
            return
        
        
        # pure pursuit
        yaw = euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])[2]
        alpha = np.arctan2(self.goal.pose.position.y - self.current_pose.position.y, self.goal.pose.position.x - self.current_pose.position.x) - yaw
        lookahead_distance = np.hypot(
            self.goal.pose.position.y - self.current_pose.position.y, self.goal.pose.position.x - self.current_pose.position.x
        )
        steering_angle = np.arctan2(2 * self.wheelbase * np.sin(alpha), lookahead_distance)

        angular_velocity = self.steering_angle_to_angular_velocity(steering_angle)
        print(
            f"alpha: {alpha} | lookahead_distance: {lookahead_distance} | steering_angle: {steering_angle} | angular_velocity: {angular_velocity}"
        )

        # publish the message
        twist = Twist()
        vel_x = self.linear_velocity
        twist.linear.x = float(vel_x)
        twist.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist)

    def steering_angle_to_angular_velocity(self, steering_angle):
        angular_velocity = np.tan(steering_angle) * self.linear_velocity / self.wheelbase
        return float(angular_velocity)


def main():
	rclpy.init()
	node_controller = Controller()
	rclpy.spin(node_controller)
	node_controller.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
    
