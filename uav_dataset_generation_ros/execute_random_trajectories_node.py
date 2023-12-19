import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, Path

from .trajectories import Circle3D, Infinity3D

from visualization_msgs.msg import Marker

from mavros_msgs.msg import State, PositionTarget

from time import time, sleep

import csv


class OffboardControl(Node):

    def __init__(self):
        super().__init__('random_trajectories_node')

        self.declare_parameter('system_id', 1)
        self.sys_id_ = self.get_parameter('system_id').get_parameter_value().integer_value

        self.declare_parameter('radius_bounds', [1.0,5.0])
        self.radius_bounds_ = self.get_parameter('radius_bounds').get_parameter_value().double_array_value

        self.declare_parameter('omega_bounds', [0.5, 1.0])
        self.omega_bounds_ = self.get_parameter('omega_bounds').get_parameter_value().double_array_value

        self.declare_parameter('xyz_bound_min', [-40.0, -40.0, 1.0])
        self.xyz_bound_min_ = self.get_parameter('xyz_bound_min').get_parameter_value().double_array_value

        self.declare_parameter('xyz_bound_max', [40.0, 40.0, 20.0])
        self.xyz_bound_max_ = self.get_parameter('xyz_bound_max').get_parameter_value().double_array_value

        # Number of random trajectories
        self.declare_parameter('num_traj', 100)
        self.num_traj_ = self.get_parameter('num_traj').get_parameter_value().integer_value

        self.declare_parameter('traj_duration', 10.0)
        self.traj_duration_ = self.get_parameter('traj_duration').get_parameter_value().double_value

        self.declare_parameter('traj_2D', False)
        self.traj_2D_ = self.get_parameter('traj_2D').get_parameter_value().bool_value

        self.declare_parameter('traj_directory', '/home/user/shared_volume/gazebo_trajectories/')
        self.traj_directory_ = self.get_parameter('traj_directory').get_parameter_value().string_value

        self.declare_parameter('file_name', 'gazebo_trajectory')
        self.file_name_ = self.get_parameter('file_name').get_parameter_value().string_value

        self.traj_objects_=[]
        self.traj_objects_.append(Circle3D(np.array([0,0,1]), np.array([0,0,1]), radius=1, omega=0.5))
        self.traj_objects_.append(Infinity3D(np.array([0,0,1]), np.array([0,0,1]), radius=1, omega=0.5))

        # Generate random parameters
        self.random_traj_params_ = self.generateRandomParameters()
        self.get_logger().info('Random trajectory parameterts are generated.')

        self.traj_counter_= 1
        self.param_counter_ = 0
        self.traj_type_counter_ = 0

        self.traj_start_t_ = time()
        self.current_traj_duration_ = 0
        self.current_traj_params_ = []

        self.reached_first_point_ = False
        self.first_point_t_ = Clock().now().nanoseconds/1000/1000/1000

        self.csv_dir_ = self.traj_directory_
        self.csv_file_ = self.csv_dir_+ self.file_name_+'_1.csv'
        # Open the CSV file
        self.file_ = open(self.csv_file_, 'w', newline='')
        self.csv_writer_ = csv.writer(self.file_)
        # Write the header
        self.csv_writer_.writerow(["timestamp", "tx", "ty", "tz"])
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_volatile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odom_ = Odometry() # latest odom

        self.status_sub_ = self.create_subscription(
            State,
            'mavros/state',
            self.vehicleStatusCallback,
            qos_profile_transient)
        
        self.odom_sub_ = self.create_subscription(
            Odometry,
            'mavros/local_position/odom',
            self.odomCallback,
            qos_profile_sensor_data)
        
        self.vehicle_path_pub_ = self.create_publisher(Path, 'offboard_visualizer/vehicle_path', 10)
        self.setpoint_path_pub_ = self.create_publisher(Path, 'offboard_visualizer/setpoint_path', 10)

        self.setopint_pub_ = self.create_publisher(PositionTarget, 'mavros/setpoint_raw/local', qos_profile_sensor_data)

        # Setpoint frequency
        timer_period = 0.02  # seconds
        self.cmd_timer_ = self.create_timer(timer_period, self.cmdloopCallback)

        self.offboard_setpoint_counter_ = 0

        self.is_armed_ = False
        self.dt_ = timer_period

        self.vehicle_path_msg_ = Path()
        self.setpoint_path_msg_ = Path()

    def __del__(self):
        self.file_.close()

    def vehicleStatusCallback(self, msg: State):
        self.is_armed_ = msg.armed

    def odomCallback(self, msg: Odometry):
        self.odom_ = msg

    def create_arrow_marker(self, id, tail, vector):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = self.odom_.header.frame_id
        # msg.header.stamp = Clock().now().nanoseconds / 1000
        msg.ns = 'arrow'
        msg.id = id
        msg.type = Marker.ARROW
        msg.scale.x = 0.1
        msg.scale.y = 0.2
        msg.scale.z = 0.0
        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0.0
        msg.color.a = 1.0
        dt = 0.3
        tail_point = Point()
        tail_point.x = tail[0]
        tail_point.y = tail[1]
        tail_point.z = tail[2]
        head_point = Point()
        head_point.x = tail[0] + dt * vector[0]
        head_point.y = tail[1] + dt * vector[1]
        head_point.z = tail[2] + dt * vector[2]
        msg.points = [tail_point, head_point]
        return msg

   
    def generateRandomNormalVector(self):
        vector = np.random.normal(size=3)
        if np.linalg.norm(vector) < 1e-6 or self.traj_2D_:
            return np.array([0, 0, 1])
        return vector

    def generateRandomCenter(self):
        xy_min, z_min = self.xyz_bound_min_[:2], self.xyz_bound_min_[2]
        xy_max, z_max = self.xyz_bound_max_[:2], self.xyz_bound_max_[2]
        x_y = np.random.uniform(xy_min, xy_max, 2)
        z = np.random.uniform(z_min, z_max)
        return np.array([*x_y, z])

    def generateRandomOmega(self):
        w_min, w_max = self.omega_bounds_[0], self.omega_bounds_[1]
        if abs(w_min) < 0.5:
            w_min = np.sign(w_min) * 0.5
        if abs(w_max) < 0.5:
            w_max = np.sign(w_max) * 0.5

        return np.random.uniform(w_min, w_max)

    def generateRandomRadius(self):
        r_min, r_max = self.radius_bounds_[0], self.radius_bounds_[1]
        return np.random.uniform(r_min, r_max)

    def generateRandomParameters(self):
        parameters = []
        for _ in range(self.num_traj_):
            params = {
                'normal_vector': self.generateRandomNormalVector(),
                'center': self.generateRandomCenter(),
                'omega': self.generateRandomOmega(),
                'radius': self.generateRandomRadius()
            }
            parameters.append(params)
        return parameters

    def cmdloopCallback(self):

        if self.param_counter_ >= len(self.random_traj_params_):
            self.param_counter_=0
            self.traj_type_counter_ +=1

        if self.traj_type_counter_ >= len(self.traj_objects_):
            self.get_logger().info(f'All trajectories are completed.')
            
            point=np.array([0.0, 0.0, 1.0])
            dist = np.sqrt((point[0] - self.odom_.pose.pose.position.x)**2 + (point[1] - self.odom_.pose.pose.position.y)**2 + (point[2] - self.odom_.pose.pose.position.z)**2)
            if (dist > 0.5):
                self.get_logger().info(f'Going to the (0,0,1) position.')

            setpoint_msg = PositionTarget()
            setpoint_msg.header.stamp = self.get_clock().now().to_msg()
            setpoint_msg.header.frame_id = self.odom_.header.frame_id
            setpoint_msg.coordinate_frame= PositionTarget.FRAME_LOCAL_NED
            setpoint_msg.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                        PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ+ PositionTarget.IGNORE_YAW_RATE
            setpoint_msg.position.x = point[0]
            setpoint_msg.position.y = point[1]
            setpoint_msg.position.z = point[2]
            # yaw  = atan(d_y, d_x)
            yaw = np.arctan2(point[1] - self.odom_.pose.pose.position.y, point[0] - self.odom_.pose.pose.position.x)
            setpoint_msg.yaw = yaw

            self.setopint_pub_.publish(setpoint_msg)
            dist = np.sqrt((point[0] - self.odom_.pose.pose.position.x)**2 + (point[1] - self.odom_.pose.pose.position.y)**2 + (point[2] - self.odom_.pose.pose.position.z)**2)

            return

        
        traj_params = self.random_traj_params_[self.param_counter_]
        traj_type = self.traj_objects_[self.traj_type_counter_]
        traj_type.updateParameters(traj_params['normal_vector'], traj_params['center'], traj_params['radius'], traj_params['omega'])
        traj_duration = traj_type.timeToCompleteFullTrajectory()
        # self.get_logger().info(f'Estimated duration of trajectory # {self.traj_counter_} = {traj_duration} seconds.')

        if (time() - self.traj_start_t_) >= traj_duration:
            self.get_logger().info(f'Completed trajectory # {self.traj_counter_}.')
            self.param_counter_ += 1
            self.traj_counter_ += 1
            self.traj_start_t_ = time()
            self.reached_first_point_ = False
            self.first_point_t_ = Clock().now().nanoseconds/1000/1000/1000
            self.get_logger().info(f'Starting trajectory # {self.traj_counter_}.')
            # TODO close file of this trajectory and open a new one to record actual positions
            self.file_.close()
            self.csv_file_ = self.csv_dir_+f'{self.file_name_}_{self.traj_counter_}.csv'
            # Open the CSV file
            self.file_ = open(self.csv_file_, 'w', newline='')
            self.csv_writer_ = csv.writer(self.file_)
            # Write the header
            self.csv_writer_.writerow(["timestamp", "tx", "ty", "tz"])
            return

        if not self.reached_first_point_:
            # First go to the first waypoint before starting the trajectory
            self.get_logger().info(f'Going to the first point of trajectory # {self.traj_counter_}.')
            point = traj_type.generate_trajectory_setpoint(self.first_point_t_)
            if point[2]< self.xyz_bound_min_[2]:
                point[2] = self.xyz_bound_min_[2]
            dist = np.sqrt((point[0] - self.odom_.pose.pose.position.x)**2 + (point[1] - self.odom_.pose.pose.position.y)**2 + (point[2] - self.odom_.pose.pose.position.z)**2)
            setpoint_msg = PositionTarget()
            setpoint_msg.header.stamp = self.get_clock().now().to_msg()
            setpoint_msg.header.frame_id = self.odom_.header.frame_id
            setpoint_msg.coordinate_frame= PositionTarget.FRAME_LOCAL_NED
            setpoint_msg.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                        PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ+ PositionTarget.IGNORE_YAW_RATE
            setpoint_msg.position.x = point[0]
            setpoint_msg.position.y = point[1]
            setpoint_msg.position.z = point[2]
            # yaw  = atan(d_y, d_x)
            yaw = np.arctan2(point[1] - self.odom_.pose.pose.position.y, point[0] - self.odom_.pose.pose.position.x)
            setpoint_msg.yaw = yaw

            self.setopint_pub_.publish(setpoint_msg)

            if dist < 0.5:
                self.first_point_t_ = Clock().now().nanoseconds/1000/1000/1000
                self.reached_first_point_ = True
            self.traj_start_t_ = time()
            return
            # self.get_logger().info(f'Reached the first point of trajectory # {self.traj_counter_}.')
            # self.get_logger().info(f'Continue trajectory # {self.traj_counter_}.')
        

        
        self.get_logger().info(f'Executing trajectory # {self.traj_counter_}.')
        t_now = Clock().now()
        point = traj_type.generate_trajectory_setpoint(t_now.nanoseconds / 1000/1000/1000)
        if point[2]< self.xyz_bound_min_[2]:
            point[2] = self.xyz_bound_min_[2]

        setpoint_msg = PositionTarget()
        setpoint_msg.header.stamp = self.get_clock().now().to_msg()
        setpoint_msg.header.frame_id = self.odom_.header.frame_id
        setpoint_msg.coordinate_frame= PositionTarget.FRAME_LOCAL_NED
        setpoint_msg.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ+ PositionTarget.IGNORE_YAW_RATE
        setpoint_msg.position.x = point[0]
        setpoint_msg.position.y = point[1]
        setpoint_msg.position.z = point[2]
        # yaw  = atan(d_y, d_x)
        yaw = np.arctan2(point[1] - self.odom_.pose.pose.position.y, point[0] - self.odom_.pose.pose.position.x)
        setpoint_msg.yaw = yaw

        self.setopint_pub_.publish(setpoint_msg)
        
        # Publish time history of the vehicle path
        vehicle_pose_msg = PoseStamped()
        vehicle_pose_msg.header = self.odom_.header
        vehicle_pose_msg.pose.position = self.odom_.pose.pose.position
        vehicle_pose_msg.pose.orientation = self.odom_.pose.pose.orientation
        self.vehicle_path_msg_.header = self.odom_.header
        self.vehicle_path_msg_.poses.append(vehicle_pose_msg)
        if (len(self.vehicle_path_msg_.poses) > 500):
            self.vehicle_path_msg_.poses.pop(0)

        self.vehicle_path_pub_.publish(self.vehicle_path_msg_)

        # Publish time history of the vehicle path
        setpoint_pose_msg = PoseStamped()
        setpoint_pose_msg.header.frame_id = self.odom_.header.frame_id
        setpoint_pose_msg.header.stamp = self.get_clock().now().to_msg()
        setpoint_pose_msg.pose.position.x = point[0]
        setpoint_pose_msg.pose.position.y = point[1]
        setpoint_pose_msg.pose.position.z = point[2]
        self.setpoint_path_msg_.header = setpoint_pose_msg.header
        self.setpoint_path_msg_.poses.append(setpoint_pose_msg)
        if (len(self.setpoint_path_msg_.poses) > 500):
            self.setpoint_path_msg_.poses.pop(0)

        self.setpoint_path_pub_.publish(self.setpoint_path_msg_)

        t = self.odom_.header.stamp.sec + self.odom_.header.stamp.nanosec * 1e-9
        tx = self.odom_.pose.pose.position.x
        ty = self.odom_.pose.pose.position.y
        tz = self.odom_.pose.pose.position.z
        # TODO Save actual poision in CSV file
        self.csv_writer_.writerow([t,tx,ty,tz])
            

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()