#!/usr/bin/python3

#ros package
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster
#rclpy libraries
import rclpy
from rclpy.node import Node

#other libraries
from math import sin, cos, atan2
from std_msgs.msg import Int64MultiArray,Float32MultiArray
import math


class OdomCalculator(Node):
    def __init__(self):
        super().__init__('odom_calculator')
        self.subscription_joint = self.create_subscription(Int64MultiArray, 'joint_motor', self.joint_state_callback, 10)
        self.subscription_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.motor_vel_publisher = self.create_publisher(Int64MultiArray, 'motor_vel', 10)
        self.motor_pos_publisher = self.create_publisher(Float32MultiArray, 'motor_pos', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        #self.timer = self.create_timer(0.1,self.odom_callback)

        self.m1_linear_pos = 0
        self.m1_linear_vel = 0
        self.m1_angular_vel = 0
        self.m2_linear_pos = 0
        self.m2_linear_vel = 0
        self.m2_angular_vel = 0
        self.robot_vel = 0
        self.robot_omega = 0

        self.m2_pos = 0
        self.m2_spd = 0

        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.theta = 0
        self.x = 0
        self.y = 0

        self.wheel_base = 0.89
        self.wheel_diametor = 0.15
        self.motor_gear = 30
        self.motor_tick = 36000
        self.get_logger().info("Published Odometry")



    def joint_state_callback(self, msg):
        #self.get_logger().info(f'Received joint_states: {msg.data}')

        m1_pos = msg.data[0]
        m1_spd = msg.data[1]
        m2_pos = msg.data[2]
        m2_spd = msg.data[3]
        
        self.m1_linear_pos = self.cal_position(m1_pos)
        self.m1_linear_vel,self.m1_angular_vel = self.cal_velocity(m1_spd)
        self.m2_linear_pos = self.cal_position(m2_pos)
        self.m2_linear_vel,self.m2_angular_vel = self.cal_velocity(m2_spd)

        motor_pos = Float32MultiArray()
        motor_pos.data = [self.m1_linear_pos , self.m2_linear_pos]
        self.motor_pos_publisher.publish(motor_pos)
        # self.get_logger().info(f'M1_pos_linear : {self.m1_linear_pos}')
        # self.get_logger().info(f'M2_pos_linear : {self.m2_linear_pos}')

        # self.get_logger().info(f'm1_linear_vel : {self.m1_linear_vel}')
        # self.get_logger().info(f'm1_angular_vel : {self.m1_angular_vel}')

        # self.get_logger().info(f'm2_linear_vel : {self.m2_linear_vel}')
        # self.get_logger().info(f'm2_angular_vel : {self.m2_angular_vel}')

        self.robot_vel,self.robot_omega = self.forward_kinematics(self.m1_linear_vel,self.m1_angular_vel,self.m2_linear_vel,self.m2_angular_vel)

        #self.get_logger().info(f'robot_vel : {self.robot_vel}')
        # self.get_logger().info(f'robot_omega : {robot_omega}')

        # self.get_logger().info(f'Robot_x : {self.x}')
        # self.get_logger().info(f'Robot_y : {self.y}')

        self.update_encoder_data(self.m1_linear_pos,self.m2_linear_pos)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        #Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self.get_quaternion(self.theta)
         # Set the velocity
        odom_msg.twist.twist.linear.x = float(self.robot_vel)
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = float(self.robot_omega)
       

        #self.odom_publisher.publish(odom_msg)#--------------------------------------------------
        # self.get_logger().info("Published Odometry")

        self.publish_transform()
    
    def cmd_vel_callback(self, msg):
        #self.get_logger().info(f'Received cmd_vel: {msg}')
        linear_vel = float(msg.linear.x)
        angular_vel = float(msg.angular.z)
        m1_vel = (linear_vel - (angular_vel * self.wheel_base / 2.0))
        m2_vel = (linear_vel + (angular_vel * self.wheel_base / 2.0))
        # self.get_logger().info(f'm1_vel : {m1_vel}')
        # self.get_logger().info(f'm2_vel : {m2_vel}')
        m1_vel_rpm = int(m1_vel *self.motor_gear* 60.0 / (math.pi * self.wheel_diametor))
        m2_vel_rpm = int(m2_vel *self.motor_gear* 60.0 / (math.pi * self.wheel_diametor))
        #self.get_logger().info(f'm1_vel_rpm : {m1_vel_rpm}')
        #self.get_logger().info(f'm2_vel_rpm : {m2_vel_rpm}')
        motor_vel = Int64MultiArray()
        motor_vel.data = [m1_vel_rpm,m2_vel_rpm]
        self.motor_vel_publisher.publish(motor_vel)
        #self.get_logger().info("Published motor_vel")

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        # Set translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        # self.get_logger().info(f'self.x : {self.x}')

        # Set rotation
        t.transform.rotation = self.get_quaternion(self.theta)

        # Broadcast TF
        self.tf_broadcaster.sendTransform(t)
    # def update_encoder_data(self, left_encoder, right_encoder):
    #     delta_distance =(left_encoder + left_encoder)/2
    #     delta_theta = (right_encoder - left_encoder ) / self.wheel_base
    #     # self.get_logger().info(f'delta_theta : {delta_theta}')
    #     # self.x = delta_distance * math.cos(delta_theta)
    #     # self.y = delta_distance * math.sin(delta_theta)
    #     self.x = delta_distance * math.cos(self.theta + delta_theta / 2)
    #     self.y = delta_distance * math.sin(self.theta + delta_theta / 2)
    #     self.theta = delta_theta
    def update_encoder_data(self, left_encoder, right_encoder):
        if abs(left_encoder - self.last_left_encoder) < 50000:
            delta_left = left_encoder - self.last_left_encoder
            self.last_left_encoder = left_encoder
        if abs(right_encoder - self.last_right_encoder) < 50000:
            delta_right = right_encoder - self.last_right_encoder
            self.last_right_encoder = right_encoder

        delta_distance = (delta_left + delta_right)/2
        delta_theta = (delta_right - delta_left) / self.wheel_base
        # self.get_logger().info(f'delta_distance : {delta_distance}')

        self.x += delta_distance * math.cos(self.theta + delta_theta/2)
        self.y += delta_distance * math.sin(self.theta + delta_theta/2)
        self.theta += delta_theta


    def get_quaternion(self, angle):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=sin(angle / 2),
            w=cos(angle / 2)
        )
    def cal_position(self, pos):
        pos_round = (pos /self.motor_gear)/self.motor_tick
        #self.get_logger().info(f'pos_round : {pos_round}')
        pos_linear = pos_round * 2 * math.pi * (self.wheel_diametor/2) # m unit //why data diif /2
        return pos_linear
    def cal_velocity(self, vel):
        rps = vel/60/self.motor_gear
        linear_velocity = (self.wheel_diametor) * math.pi * rps # m/s unit
        angular_velocity = linear_velocity/(self.wheel_diametor/2) #  unit

        return linear_velocity,angular_velocity
    def forward_kinematics(self,v1,w1,v2,w2):
    # Convert motor velocities to angular velocities

        # Calculate linear and angular velocities
        v = (v1+v2)/2
        omega = ((self.wheel_diametor/2)/ self.wheel_base) * (w2 - w1)
        return v, omega

def main(args=None):
    rclpy.init(args=args)
    node = OdomCalculator()
    try:
        while rclpy.ok(): # while the node isn't shut down
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node has stopped cleanly.')
    except SystemExit:
        node.get_logger().info('Node is complete.')
    except BaseException as exc:
        type = exc.__class__.__name__
        node.get_logger().error(f'{type} exception in node has occured.')
        raise # raise without argument = raise the last exception
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
