import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):
    # Check the closest distance in the LaserScan data
    closest_distance = min(msg.ranges)
    
    # Adjust motor commands based on obstacle distance
    if closest_distance < 0.5:  # Set your desired obstacle distance threshold
        # Obstacle detected, perform obstacle avoidance
        print("Obstacle detected! Turning...")
        # Print motor commands (you can replace this with your actual motor control logic)
        print("Left Motor: Forward")
        print("Right Motor: Backward")
    else:
        # No obstacle, move forward
        print("No obstacle. Moving forward...")
        # Print motor commands (you can replace this with your actual motor control logic)
        print("Left Motor: Forward")
        print("Right Motor: Forward")

def main():
    rclpy.init()

    node = rclpy.create_node('motor_command_listener')

    # Subscribe to the "scan" topic
    subscription = node.create_subscription(
        LaserScan,
        'scan',
        callback,
        10  # QoS profile depth
    )
    subscription  # Prevent unused variable warning

    try:
        # Spin the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node shutting down...")
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
