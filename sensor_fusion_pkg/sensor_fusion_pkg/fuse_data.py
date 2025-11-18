import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


class Sensor_Fusion_Node(Node):
    def __init__(self):
        super().__init__('fused_data')
        # Settign up subscribers for IMU and Depth data
        self.imu_subscriber = self.create_subscription(Imu,'/imu/data',self.imu_callback,10)
        self.depth_subscriber = self.create_subscription(Float32,'/depth',self.depth_callback,10)
        
        # Publisher for vertical velocity
        self.velocity_publisher = self.create_publisher(Float32,'/vertical_velocity',10)

        self.timer = self.create_timer(0.005, self.timer_callback)
        
        # Initialize variables to store last depth and time
        self.prev_depth = None
        self.prev_depth_time = None

        self.vertical_vel_imu = 0.0
        self.vertical_vel_depth = 0.0

        self.prev_imu_time = None

        # Log node startup
        self.get_logger().info('Sensor Fusion Node has been started...')


    def imu_callback(self, msg):

        #extract vertical acceleration from IMU data
        acc_z = msg.linear_acceleration.z - 9.81  # Subtract gravity

        # Get current time, """USING SYSTEM TIME for simplicity not message timestamps"""
        current_imu_time = time.time()

        # Integrate acceleration to estimate vertical velocity       
        if self.prev_imu_time is not None:
            dt = current_imu_time - self.prev_imu_time
            self.vertical_vel_imu += acc_z * dt    
            
        self.prev_imu_time = current_imu_time

        # self.get_logger().info(f'Received IMU data: {msg}')


    def depth_callback(self, msg):
        # Extract current depth from Depth data
        current_depth = msg.data
        t = time.time()

        # Calculate vertical velocity using change in depth over time
        if self.prev_depth is not None:
            dt = t - self.prev_depth_time
            self.vertical_vel_depth = (current_depth - self.prev_depth)/ dt

        self.prev_depth_time = t
        self.prev_depth = current_depth


        # self.get_logger().info(f'Received Depth data: {msg}')

    def timer_callback(self):
        # Simple average fusion of both velocity estimates
        fused_velocity = Float32()

        # Fusion both velocity based on weights
        fused_velocity.data = (0.5 * self.vertical_vel_imu) + (0.5 * self.vertical_vel_depth)

        # Publish the fused vertical velocity
        self.velocity_publisher.publish(fused_velocity)

        self.get_logger().info(f'Published Fused Vertical Velocity at 200 Hz: {fused_velocity.data}')


def main(args=None):
    rclpy.init(args=args)
    fused_data = Sensor_Fusion_Node()
    rclpy.spin(fused_data)
    fused_data.destroy_node()
    rclpy.shutdown()    


if __name__ == '__main__':
    main()


