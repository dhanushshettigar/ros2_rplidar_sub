import rclpy
from sensor_msgs.msg import LaserScan
import pygame
from math import cos, sin, pi, floor, isinf

# Pygame setup with larger window size
pygame.init()
width, height = 800, 600
lcd = pygame.display.set_mode((width, height))
pygame.mouse.set_visible(False)
lcd.fill((0, 0, 0))
pygame.display.update()

# Global variable for max_distance
max_distance = 0

# Callback function for processing data from the scan topic
def lidar_callback(msg):
    global max_distance
    max_distance = 0  # Initialize max_distance to zero
    scan_data = [0] * 360
    lcd.fill((0, 0, 0))

    for i, distance in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        scan_data[min([359, floor(angle * 180.0 / pi)])] = distance

    for angle in range(360):
        distance = scan_data[angle]
        if not isinf(distance):  # Skip processing if the distance is infinity
            if distance > 0:  # ignore initially ungathered data points
                max_distance = max([min([5000, distance]), max_distance])
                radians = angle * pi / 180.0
                x = distance * cos(radians)
                y = distance * sin(radians)
                # Adjust coordinates to center the points in the larger window
                point = (width // 2 + int(x / max_distance * (width - 1) / 2),
                         height // 2 + int(y / max_distance * (height - 1) / 2))
                lcd.set_at(point, pygame.Color(255, 255, 255))

    pygame.display.update()

def main():
    rclpy.init()

    node = rclpy.create_node('lidar_listener')

    # Subscribe to the "scan" topic
    subscription = node.create_subscription(
        LaserScan,
        'scan',
        lidar_callback,
        10  # QoS profile depth
    )
    subscription  # Prevent unused variable warning

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            pygame.time.delay(10)  # Add a small delay to control the frame rate
    except KeyboardInterrupt:
        print("Node shutting down...")
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
