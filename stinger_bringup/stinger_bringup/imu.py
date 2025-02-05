'''
Adafruit TDK InvenSense ICM-20948 9-DoF IMU (4554)
This node is publish topic /imu/data

1. linear acceleration [m/s^2]
2. angular velocity [rad/s]
3. compass

Note the imu data is publishing in the 'imu_link'. Keep that in mind with writing urdf.

Resources: https://learn.adafruit.com/adafruit-tdk-invensense-icm-20948-9-dof-imu/python-circuitpython
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import adafruit_icm20x
# import time
# from geometry_msgs.msg import Vector3

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Initialize IMU sensor over I2C
        i2c = board.I2C()
        self.imu_sensor = adafruit_icm20x.ICM20948(i2c)
        
        # Publisher for IMU data
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # Publish at 10 Hz

    def publish_imu_data(self):
        msg = Imu()
        msg.header.frame_id = "imu_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        # Read IMU sensor data
        try:
            accel_x, accel_y, accel_z = self.imu_sensor.acceleration
            gyro_x, gyro_y, gyro_z = self.imu_sensor.gyro
            mag_x, mag_y, mag_z = self.imu_sensor.magnetic
            
            # Fill message fields
            msg.linear_acceleration.x = accel_x
            msg.linear_acceleration.y = accel_y
            msg.linear_acceleration.z = accel_z

            msg.angular_velocity.x = gyro_x
            msg.angular_velocity.y = gyro_y
            msg.angular_velocity.z = gyro_z

            # Log and publish the message
            self.imu_pub.publish(msg)
            # self.get_logger().info(f"Accel: ({accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}) "
            #                         f"Gyro: ({gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}) "
            #                         f"Mag: ({mag_x:.2f}, {mag_y:.2f}, {mag_z:.2f})")
        
        except Exception as e:
            self.get_logger().error(f"Error reading IMU data: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
