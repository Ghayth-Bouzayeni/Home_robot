import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import paho.mqtt.client as mqtt


class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('command_subscriber')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.client = mqtt.Client()
        self.client.on_message = self.on_mqtt_message
        self.client.connect('localhost', 1883, 60)
        self.client.subscribe('robot/command')
        self.client.loop_start()

        self.current_position = {"x": 0.0, "y": 0.0, "yaw": 0.0}  # Robot's current position and orientation

        
        self.locations = {
            # you can adjust  it for your goal
            "kitchen": {"x": 5.3373408317584, "y": 6.604702945787636},  # you can adjust  it for your goal
           "bedroom": {"x": -3.75, "y": 3.65},  
            "safe": {"x": 0.0, "y": 0.0}  
        }

    def on_mqtt_message(self, client, userdata, msg):
        command = msg.payload.decode('utf-8')
        self.get_logger().info(f'Received command from MQTT: {command}')
        twist = Twist()

        
        if command == "move forward":
            twist.linear.x = 0.5  #
        elif command == "turn left":
            twist.angular.z = 0.5  
        elif command == "turn right":
            twist.angular.z = -0.5 
        elif command == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif command.startswith("go to "):  
            location = command.split("go to ")[1]  
            if location in self.locations:
                self.move_to_location(location)
            else:
                self.get_logger().warning(f'Unknown location: {location}')
            return
        elif command == "cancel":
            self.cancel_and_move_to_safe()
        else:
            self.get_logger().warning(f'Unknown command: {command}')
            return

        self.publisher.publish(twist)
        self.get_logger().info(f'Sent Twist message: {twist}')

    def move_to_location(self, location):
        
        coordinates = self.locations[location]

        # Create a PoseStamped message to send the robot to the goal location
        goal = PoseStamped()
        goal.header.frame_id = 'map'  # Ensure the goal frame is set correctly
        goal.pose.position.x = coordinates["x"]
        goal.pose.position.y = coordinates["y"]
        goal.pose.orientation.w = 1.0  

        # Publish the goal to the ROS navigation system (e.g., move_base)
        self.pose_publisher.publish(goal)
        self.get_logger().info(f'Sent goal to move to {location} at {coordinates}')

    def odom_callback(self, msg):
        # Extract the current position and orientation from Odometry
        self.current_position["x"] = msg.pose.pose.position.x
        self.current_position["y"] = msg.pose.pose.position.y

        # Extract orientation (convert quaternion to yaw)
        orientation = msg.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        self.current_position["yaw"] = yaw

    def euler_from_quaternion(self, x, y, z, w):
        # Convert quaternion to Euler angles (yaw, pitch, roll)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def cancel_and_move_to_safe(self):
        # Publish a stop Twist message
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.publisher.publish(stop_twist)
        self.get_logger().info('Canceled current motion')

        #Compute a nearby "safe" position
        safe_position = self.compute_safe_position()
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = safe_position["x"]
        goal.pose.position.y = safe_position["y"]
        goal.pose.orientation.w = 1.0

        # Publish the goal
        self.pose_publisher.publish(goal)
        self.get_logger().info(f'Moving to dynamically calculated safe position: {safe_position}')

    def compute_safe_position(self):
        #calculate a point 1 meter behind the robot
        x = self.current_position["x"]
        y = self.current_position["y"]
        yaw = self.current_position["yaw"]

        #offset by 1 meter in the direction opposite to the robot's yaw
        safe_x = x - 0.5 * math.cos(yaw)
        safe_y = y - 0.5 * math.sin(yaw)

        return {"x": safe_x, "y": safe_y}


def main(args=None):
    rclpy.init(args=args)
    node = CommandSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()