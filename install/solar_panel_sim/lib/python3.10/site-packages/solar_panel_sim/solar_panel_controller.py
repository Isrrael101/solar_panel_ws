import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import pandas as pd
from datetime import datetime
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class SolarPanelController(Node):
    def __init__(self):
        super().__init__('solar_panel_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/solar_panel/joint_trajectory_controller/joint_trajectory', 10)
        self.joint_state_subscriber_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Declare and get parameters
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        if not config_file:
            self.get_logger().error('Config file parameter is empty')
            return

        # Load configuration
        try:
            with open(config_file, 'r') as file:
                self.config = yaml.safe_load(file)['/**']['ros__parameters']
        except FileNotFoundError:
            self.get_logger().error(f'Config file not found: {config_file}')
            return
        except KeyError:
            self.get_logger().error(f'Invalid config file structure: {config_file}')
            return
        
        # Get the package share directory
        pkg_share = get_package_share_directory('solar_panel_sim')
        
        # Construct the full path to the CSV file
        csv_file = os.path.join(pkg_share, 'resource', 'solar_parameters_2024.csv')
        self.get_logger().info(f'Loading CSV file from: {csv_file}')
        
        if not os.path.exists(csv_file):
            self.get_logger().error(f'CSV file not found: {csv_file}')
            return

        start_date = self.config['start_date']
        end_date = self.config['end_date']
        update_rate = self.config['update_rate']
        
        try:
            self.df = pd.read_csv(csv_file)
            self.get_logger().info(f'CSV file loaded successfully. Shape: {self.df.shape}')
            self.get_logger().info(f'Columns: {self.df.columns.tolist()}')
            
            self.df['FechaHora'] = pd.to_datetime(self.df['FechaHora'])
            self.df = self.df[(self.df['FechaHora'] >= start_date) & (self.df['FechaHora'] <= end_date)]
            
            self.get_logger().info(f'Filtered data shape: {self.df.shape}')
        except Exception as e:
            self.get_logger().error(f'Error processing CSV file: {str(e)}')
            return
        
        self.current_index = 0
        self.timer = self.create_timer(update_rate, self.timer_callback)

        # Configure PID controllers
        self.azimuth_pid = PIDController(self.config['control']['azimuth']['p'],
                                         self.config['control']['azimuth']['i'],
                                         self.config['control']['azimuth']['d'])
        self.elevation_pid = PIDController(self.config['control']['elevation']['p'],
                                           self.config['control']['elevation']['i'],
                                           self.config['control']['elevation']['d'])

        # Initialize current joint positions
        self.current_azimuth = 0.0
        self.current_elevation = 0.0

        # Add low-pass filters for smoothing
        self.azimuth_filter = LowPassFilter(0.1)  # Adjust alpha as needed
        self.elevation_filter = LowPassFilter(0.1)  # Adjust alpha as needed

    def joint_state_callback(self, msg):
        # Update current joint positions
        if 'azimuth_joint' in msg.name:
            self.current_azimuth = msg.position[msg.name.index('azimuth_joint')]
        if 'elevation_joint' in msg.name:
            self.current_elevation = msg.position[msg.name.index('elevation_joint')]

    def timer_callback(self):
        if self.current_index < len(self.df):
            row = self.df.iloc[self.current_index]
            target_azimuth = row['Azimut_Grados']
            target_elevation = row['Elevacion_Grados']
            
            azimuth_control = self.azimuth_pid.update(target_azimuth, self.current_azimuth)
            elevation_control = self.elevation_pid.update(target_elevation, self.current_elevation)

            # Apply low-pass filter for smoother control
            azimuth_control = self.azimuth_filter.update(azimuth_control)
            elevation_control = self.elevation_filter.update(elevation_control)

            msg = Float64MultiArray()
            msg.data = [azimuth_control, elevation_control]
            self.publisher_.publish(msg)
            
            self.current_index += 1
            if self.current_index % 100 == 0:
                self.get_logger().info(f'Processing data point {self.current_index}/{len(self.df)}')
        else:
            self.get_logger().info('Simulation completed')
            self.timer.cancel()

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def update(self, setpoint, current_value):
        error = setpoint - current_value
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value

def main(args=None):
    rclpy.init(args=args)
    solar_panel_controller = SolarPanelController()
    rclpy.spin(solar_panel_controller)
    solar_panel_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()