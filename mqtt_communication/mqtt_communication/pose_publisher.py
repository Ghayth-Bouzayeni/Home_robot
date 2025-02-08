import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        
        # initialize MQTT client and connect to broker
        self.client = mqtt.Client()
        self.client.connect("localhost", 1883, 60)  # Connect to local MQTT broker

        # initialize ROS 2 publisher for /pose topic
        self.publisher = self.create_publisher(PoseStamped, '/pose', 10)

        # Timer to publish pose at regular intervals
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1-second interval

        # subscribe to the odom topic to get robot pose
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.listener_callback, 10)

        self.latest_pose = None

    def timer_callback(self):
        if self.latest_pose is not None:
            # Create a new PoseStamped message
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'  # or 'odom', depending on your frame

            # Update the pose with the latest odometry data
            msg.pose.position.x = self.latest_pose.pose.pose.position.x
            msg.pose.position.y = self.latest_pose.pose.pose.position.y
            msg.pose.position.z = self.latest_pose.pose.pose.position.z
            msg.pose.orientation = self.latest_pose.pose.pose.orientation

            # Publish pose to ROS topic
            self.publisher.publish(msg)
            self.get_logger().info(f'Published pose to ROS topic: {msg}')

            # Publish pose data to MQTT broker
            pose_data = f"Position: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})"
            self.client.publish("/pose", pose_data)
            self.get_logger().info(f'Published pose to MQTT: {pose_data}')

        else:
            self.get_logger().warn('No pose data received yet.')

    def listener_callback(self, msg):
        self.latest_pose = msg

        # lil debugging
        pose_data = f"Received pose: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z})"
        self.get_logger().info(f'Updated pose: {pose_data}')

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
