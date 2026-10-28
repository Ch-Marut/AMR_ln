#!/usr/bin/python3

#ros package

#rclpy libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray,Float32MultiArray
from geometry_msgs.msg import Pose,PoseWithCovarianceStamped
from rclpy.parameter import Parameter
from rclpy.client import Client
from rcl_interfaces.srv import SetParameters
#other libraries
from pyModbusTCP.client import ModbusClient

import math
#from command_pose.srv import Nav2GoPose
from amr_service.srv import NavInitPose,NavToPose,NavCancleTask,NavThroughPose

class Plc_communication(Node):
    def __init__(self):
        timer_period = 0.01
        super().__init__('plc_communication')
        # custom attributes
        self.get_logger().info(f'{self.get_name()} has started.')
        self.publisher = self.create_publisher(Int64MultiArray, 'joint_motor', 10)
        self.publisher_plc_io = self.create_publisher(Int64MultiArray, 'plc_io', 10)
        self.publisher_param = self.create_publisher(Float32MultiArray, 'param_state', 10)
        
        self.subscription_motot_vel = self.create_subscription(Int64MultiArray, 'motor_vel', self.motor_vel_callback, 10)
        self.subscription_motot_pos = self.create_subscription(Float32MultiArray, 'motor_pos', self.motor_pos_callback, 10)

        self.client_ini_pose =self.create_client(NavInitPose,'nav2_init_pose')
        self.client_to_pose =self.create_client(NavToPose,'nav2_to_pose')
        self.client_through_pose =self.create_client(NavThroughPose,'nav2_through_pose')
        self.client_cancle_task =self.create_client(NavCancleTask,'nav2_cancle_task')
        

        self.subscriptions_amcl = self.create_subscription(PoseWithCovarianceStamped,'amcl_pose',self.amcl_callback,10)

        self.timer = self.create_timer(timer_period,self.timer_callback)

        self.c = ModbusClient(host="192.168.250.50", port=502, unit_id=1, auto_open=True)



        self.c.write_single_coil(0x2006,True)

        self.amcl_x = 0
        self.amcl_y = 0
        self.amcl_w = 0 
        self.future = None
        self.flag = True
        self.target_posses = Pose()
        
        
        self.position_threshold = 0
        self.position_distance = 0
        self.yaw_data = 0
        self.target_yaw_data = 0
        self.prev_confirm_target_pos = False
        self.prev_confirm_target_poses = False
        self.prev_init_pos = False
        self.prev_cancle_task = False

    def timer_callback(self):
        
        #c.write_single_register(1,0)
        

       
        
        #self.get_logger().info(f"Published param_state: robot_radius={robot_radius}, inflation_radius={inflation_radius}, width={width}")

        
        input_plc = self.c.read_coils(0x2000, 500) #read data M0 >>2 M1
        #regs = c.read_holding_registers(5000,10) #Read D 4 point
        reg_motor = self.c.read_holding_registers(5001, 8)
        reg_target_pos = self.c.read_holding_registers(5010, 8)
        reg_threshold_pos = self.c.read_holding_registers(5020, 1)
        reg_threshold_pos2 = self.c.read_holding_registers(5021, 1)

        robot_radius = self.c.read_holding_registers(5022, 1)
        inflation_radius = self.c.read_holding_registers(5023, 1)
        width = self.c.read_holding_registers(5024, 1)
        #self.get_logger().info(f"Published param_state: robot_radius={float(robot_radius[0])}, inflation_radius={float(inflation_radius[0])}, width={float(width[0])}")
        #self.get_logger().info(f"Published type: robot_radius={type(robot_radius[0])}, inflation_radius={inflation_radius}, width={width}")
        param_msg = Float32MultiArray()
        param_msg.data = [float(robot_radius[0]), float(inflation_radius[0]), float(width[0])]  # Add width to the message
        self.publisher_param.publish(param_msg)

        self.position_threshold = float(reg_threshold_pos[0])/1000
        self.position_threshold2 = float(reg_threshold_pos2[0])/1000

            
        #self.get_logger().info(f'Read position_threshold: {self.position_threshold}')
        
        io_plc = input_plc[200:220]
        confirm_target_pos = io_plc[0]
        confirm_init_pos = io_plc[1]
        ene_flag = io_plc[2]
        cancle_task = io_plc[3]
        confirm_target_poses = io_plc[4]
        small_size = io_plc[5]
        #self.get_logger().info(f'io_plc: {io_plc}')
        #self.get_logger().info(f'flag reading: {self.flag}')
        if reg_motor:
            #self.get_logger().info(f'data reading: {reg_motor}')
            m1_pos = twos_complement_to_decimal(reg_motor[0],reg_motor[1])
            m1_spd = complement_to_decimal(reg_motor[2])
            m2_pos = twos_complement_to_decimal(reg_motor[4],reg_motor[5])
            m2_spd = complement_to_decimal(reg_motor[6])

            # self.get_logger().info(f'm1_pos: {m1_pos}')
            # self.get_logger().info(f'm1_spd: {m1_spd}')
            # self.get_logger().info(f'm2_pos: {m2_pos}')
            # self.get_logger().info(f'm2_spd: {m2_spd}')

            data_motor = [m1_pos,m1_spd,m2_pos,m2_spd]

            joint_states_msg = Int64MultiArray()
            joint_states_msg.data =  data_motor

            self.publisher.publish(joint_states_msg)
            #self.get_logger().info("Published joint motor")
        if reg_target_pos:
            #self.get_logger().info(f'Target reading: {reg_target_pos}')
            self.target_posses.position.x = float(twos_complement_to_decimal(reg_target_pos[0],reg_target_pos[1]))/1000
            self.target_posses.position.y = float(twos_complement_to_decimal(reg_target_pos[2],reg_target_pos[3]))/1000
            self.target_posses.orientation.z = float(complement_to_decimal(reg_target_pos[4]))/1000
            self.target_posses.orientation.w = float(complement_to_decimal(reg_target_pos[6]))/1000
            #self.get_logger().info(f'Target convert: {self.target_posses}')
        else:
            self.get_logger().info('read error')
        
        if confirm_target_pos and self.flag and not self.prev_confirm_target_pos:
            self.flag = False
            #self.get_logger().info(f'flag reading: {self.flag}')
            req = NavToPose.Request()
            req.target_pose = self.target_posses
            
            #self.c.write_single_coil(0x2004,False)
            self.client_to_pose.call_async(req)
            #self.future.add_done_callback(self.service_response_callback)
        if confirm_target_poses and self.flag and not self.prev_confirm_target_poses:
            self.flag = False
            #self.get_logger().info(f'flag reading: {self.flag}')
            req = NavThroughPose.Request()
            req.target_pose = self.target_posses
            
            #self.c.write_single_coil(0x2004,False)
            self.client_through_pose.call_async(req)
            #self.future.add_done_callback(self.service_response_callback)
        if confirm_init_pos == True and not self.prev_init_pos:
            #self.get_logger().info(f'io_plc: {io_plc}')
            self.prev_init_pos = True
            req_init = NavInitPose.Request()
            req_init.target_pose = self.target_posses
            self.client_ini_pose.call_async(req_init)

        if cancle_task == True and not self.prev_cancle_task:
            #self.get_logger().info(f'io_plc: {io_plc}')
            self.prev_cancle_task = True
            req_cancle = NavCancleTask.Request()
            req_cancle.target_pose = self.target_posses
            self.client_cancle_task.call_async(req_cancle)
            
        if confirm_init_pos == False:
            self.prev_init_pos = False

        if cancle_task == False:
            self.prev_cancle_task = False

        if ene_flag == True:
            self.flag = True

        if ((self.position_distance < self.position_threshold)) :
        #self.get_logger().info('AMCL pose is close to the target pose with a small gap')
            self.c.write_single_coil(0x2004,True) #in position
            self.flag = True
        else:
        #self.get_logger().info('-AMCL pose is not close enough to the target pose')
            self.c.write_single_coil(0x2004,False) #out of position

        if ((self.position_distance < self.position_threshold2)) :
        #self.get_logger().info('AMCL pose is close to the target pose with a small gap')
            self.c.write_single_coil(0x2003,True) #in position
        else:
        #self.get_logger().info('-AMCL pose is not close enough to the target pose')
            self.c.write_single_coil(0x2003,False) #out of position
      
        self.prev_confirm_target_pos = confirm_target_pos
        self.prev_confirm_target_poses = confirm_target_poses
        #self.get_logger().info(f'confirm_target_pos: {confirm_target_pos}')
        #self.get_logger().info(f'self.flag: {self.flag}')
        #self.get_logger().info(f'self.prev_confirm_target_pos: {self.prev_confirm_target_pos}')
        #if small_size == True:

    # def service_response_callback(self, response):
    #     # This callback function will be called when the service response is received
    #     try:
    #         response = self.future.result()
    #         self.get_logger().info(f"Received response: {response.complete}")
    #         self.flag = True
            
    #     except Exception as e:
    #         self.get_logger().error(f"Service call failed: {e}")
    #     #self.get_logger().info(f"Received response: {response.complete}")

    def motor_vel_callback(self,msg):
        #self.get_logger().info(f'Received motor speed in rpm : {msg.data}')
        m1_spd_rpm = decimal_to_complement (msg.data[0])
        m2_spd_rpm = decimal_to_complement (msg.data[1])
        #self.get_logger().info('HI')
        self.c.write_multiple_registers(5100,[m1_spd_rpm,m2_spd_rpm])
    
    def motor_pos_callback(self,msg):
        #self.get_logger().info(f'Received motor speed in rpm : {msg.data}')
      
        m1_pos_rpm0,m1_pos_rpm1 = decimal_to_twos_complement(int(msg.data[0]*100))

        m2_pos_rpm0,m2_pos_rpm1 = decimal_to_twos_complement(int(msg.data[1]*100))
        #self.get_logger().info(f'Received motor pos in mm : {m1_pos_rpm,m2_pos_rpm}')
        #self.get_logger().info('HI')
        self.c.write_multiple_registers(5110,[m1_pos_rpm0,m1_pos_rpm1,m2_pos_rpm0,m2_pos_rpm1])

    def amcl_callback(self,msg):
        try:
        #self.get_logger().info(f'Recieve AMCL : = {msg.pose.pose.position.x},{msg.pose.pose.position.y},{msg.pose.pose.orientation.z},{msg.pose.pose.orientation.w}')
            # self.amcl_x = decimal_to_complement(int(msg.pose.pose.position.x*100))
            # self.amcl_y = decimal_to_complement(int(msg.pose.pose.position.y*100))
            # self.amcl_z = decimal_to_complement(int(msg.pose.pose.orientation.z*100))
            # self.amcl_w = decimal_to_complement(int(msg.pose.pose.orientation.w*100))
            amcl_x0,amcl_x1 = decimal_to_twos_complement(int(msg.pose.pose.position.x*1000))
            amcl_y0,amcl_y1 = decimal_to_twos_complement(int(msg.pose.pose.position.y*1000))
            amcl_z0,amcl_z1 = decimal_to_twos_complement(int(msg.pose.pose.orientation.z*1000))
            amcl_w0,amcl_w1 = decimal_to_twos_complement(int(msg.pose.pose.orientation.w*1000))
            # amcl_x0 = decimal_to_complement(amcl_x0)
            # amcl_x1 = decimal_to_complement(amcl_x1)
            # amcl_y0 = decimal_to_complement(amcl_y0)
            # amcl_y1 = decimal_to_complement(amcl_y1)
            # amcl_z0 = decimal_to_complement(amcl_z0)
            # amcl_z1 = decimal_to_complement(amcl_z1)
            # amcl_w0 = decimal_to_complement(amcl_w0)
            # amcl_w1 = decimal_to_complement(amcl_w1)
            # self.get_logger().info(f'amcl_pose : {amcl_x0},{amcl_x1},{amcl_y0},{amcl_y1}')
            # self.get_logger().info(f'amcl_orien : {amcl_z0},{amcl_z1},{amcl_w0},{amcl_w1}')
            # self.get_logger().info(f'target : {self.target_posses.position.x}')
            # self.get_logger().info(f'amcl_pose : {float(msg.pose.pose.position.x)}')
            self.position_distance = math.sqrt((self.target_posses.position.x - float(msg.pose.pose.position.x))**2 + (self.target_posses.position.y- float(msg.pose.pose.position.y))**2)
            
            #self.get_logger().info(f'position_distance : {position_distance}')
            #orientation_distance = math.sqrt((target_z - amcl_z)**2 + (target_w - amcl_w)**2)

            self.yaw_data = int(rad_to_ang(quaternion_to_yaw(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w))*10)
            self.target_yaw_data = int(rad_to_ang(quaternion_to_yaw(self.target_posses.orientation.z,self.target_posses.orientation.w))*10)
            #self.get_logger().info(f'Yaw_data : {self.yaw_data}')

            turn_direction = compute_turn_direction(quaternion_to_yaw(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),quaternion_to_yaw(self.target_posses.orientation.z,self.target_posses.orientation.w))
            #self.get_logger().info(f'turn_direction : {turn_direction}')
            self.c.write_single_coil(0x2005,turn_direction)

            self.c.write_multiple_registers(5155,[self.yaw_data,self.target_yaw_data])
            

            #self.c.write_multiple_registers(5102,[self.amcl_x,self.amcl_y,self.amcl_z,self.amcl_w])
            self.c.write_multiple_registers(5102,[amcl_x0,amcl_x1,amcl_y0,amcl_y1,amcl_z0,amcl_z1,amcl_w0,amcl_w1])
            #self.c.write_multiple_registers(5120,turn_direction)
        except ValueError as e:
            self.get_logger().error(f'Error in amcl_callback: {e}')

def quaternion_to_yaw(q_z, q_w):
    # Compute yaw angle from quaternion
    yaw = math.atan2(2*(q_w*q_z), 1 - 2*(q_z**2))
    return yaw

def rad_to_ang(rad):
    ang = rad*(180/math.pi)
    if ang < 0 : 
        ang = 360 + ang
    return ang
    
def compute_turn_direction(current_yaw, target_yaw):
  
    # Compute angle difference
    angle_difference = math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))
    
    # Determine turn direction
    if angle_difference >= 0:
        return True
    elif angle_difference < 0:
        return False
    
def twos_complement_to_decimal(data1,data2):
# Check if the number is negative

    data_b1 = format(int(data1), '016b')
    data_b2 = format(int(data2), '016b')

    binary_merged = data_b2 + data_b1

    is_negative = binary_merged[0] == '1'

# If negative, convert to positive using 2's complement
    if is_negative:
        # Invert all bits
        inverted_bits = ''.join('1' if bit == '0' else '0' for bit in binary_merged)
        # Add 1 to the inverted bits
        binary_merged = bin(int(inverted_bits, 2) + 1)[2:]

    # Convert binary to decimal
    decimal_result = int(binary_merged, 2)

    # If originally negative, make the result negative
    return -decimal_result if is_negative else decimal_result

def decimal_to_twos_complement(decimal_data):
    # Check if the number is negative
    is_negative = decimal_data < 0

    # Convert decimal to binary
    binary_merged = bin(abs(decimal_data))[2:].zfill(32)

    # If negative, convert to 2's complement
    if is_negative:
        # Invert all bits
        inverted_bits = ''.join('1' if bit == '0' else '0' for bit in binary_merged)
        # Add 1 to the inverted bits
        binary_merged = bin(int(inverted_bits, 2) + 1)[2:]

    # Split the 32-bit binary into two 16-bit parts
    data_b2 = binary_merged[:16]
    data_b1 = binary_merged[16:]

    # Convert binary to decimal for each part
    decimal_data1 = int(data_b1, 2)
    decimal_data2 = int(data_b2, 2)

    # If originally negative, make the parts negative
    # if is_negative:
    #     decimal_data1 = -decimal_data1
    #     decimal_data2 = -decimal_data2

    return decimal_data1, decimal_data2

def complement_to_decimal(data1):
# Check if the number is negative
    data1 = format(int(data1), '016b')
    is_negative = data1[0] == '1'

# If negative, convert to positive using 2's complement
    if is_negative:
        # Invert all bits
        inverted_bits = ''.join('1' if bit == '0' else '0' for bit in data1)
        # Add 1 to the inverted bits
        data1 = bin(int(inverted_bits, 2) + 1)[2:]

    # Convert binary to decimal
    decimal_result = int(data1, 2)

    # If originally negative, make the result negative
    return -decimal_result if is_negative else decimal_result
def decomplement_to_decimal(data1):
# Check if the number is negative
    data1 = format(int(data1), '016b')
    is_negative = data1 < 0 == '1'
# If negative, convert to positive using 2's complement
    if is_negative:
        # Invert all bits
        inverted_bits = ''.join('1' if bit == '0' else '0' for bit in data1)
        # Add 1 to the inverted bits
        data1 = bin(int(inverted_bits, 2) + 1)[2:]

    # Convert binary to decimal
    decimal_result = int(data1, 2)

    # If originally negative, make the result negative
    return -decimal_result if is_negative else decimal_result

def decimal_to_complement(decimal_value):
    # Check if the number is negative
    is_negative = decimal_value < 0
    # If negative, convert to positive and then to 2's complement
    if is_negative:
        # Convert to positive and get binary representation
        positive_binary = bin(abs(decimal_value))[2:]
        # Pad with zeros to make it 16 bits (adjust the number as needed)
        positive_binary = positive_binary.zfill(16)

        # Invert all bits
        inverted_bits = ''.join('1' if bit == '0' else '0' for bit in positive_binary)
        # Add 1 to the inverted bits
        complement_result = int(inverted_bits, 2) + 1

    else:
        # If positive, convert directly to binary and pad with zeros
        complement_result = decimal_value

    return complement_result
def main(args=None):
    rclpy.init(args=args)
    node = Plc_communication()
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
if __name__=='__main__':
    main()
