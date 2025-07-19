import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import can 
import struct
from time import time, sleep

from std_msgs.msg import String


class GripperCanControl(Node):

    def __init__(self):
        super().__init__('gripper_can_control')

        self.last_message_time = time()
        #odrive params
        self.node_id = 6
        self.axis = 0

        self.vel_setpoint = 10
        self.vel_limit = 24.0
        self.vel_ramp_rate = 10.0
        self.current_threshold = 2
        self.torq_setpoint = 0.03124

        self.current = 0.0
        self.current_pos = 0.0


        self.home_current_threshold = 2
        self.home_vel = 5
        self.home_pos = 0.0
        self.home_offset = 0.5
    

        #joy Mappings
        self.open_button = 1
        self.close_button = 2

        #setup can
        self.bus = can.interface.Bus(channel='can1', bustype='socketcan')
        self.can_timer = self.create_timer(0.005, self.read_can)
        #set up joy
        self.create_subscription(Joy, '/joy', self.joy_callback, 1)

        #Initialize Odrive and Gripper position 
        self.setup_controller()
        #create time out timer
        self.timer = self.create_timer(0.01, self.timer_callback)
        

    def timer_callback(self):
        #timeout period of half sec
        if abs(self.last_message_time - time()) > 0.5 and self.current < self.current_threshold:
            self.send_position(pos=self.current_pos)

    def joy_callback(self, msg):
        self.last_message_time = time()
        buttons = msg.buttons
        if buttons[self.open_button]: 
            if self.current > self.current_threshold:
                self.send_position(self.home_pos)
            elif self.current_pos >= self.home_pos:
                self.send_position(self.current_pos)
            else:
                self.send_velocity(self.vel_setpoint)


        elif buttons[self.close_button]: 
            if self.current > self.current_threshold:
                self.send_torque(self.torq_setpoint)
            else: 
                self.send_velocity(self.vel_setpoint)

    def setup_controller(self):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x07), # 0x07: Set_Axis_State
            data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
            is_extended_id=False

        ))

        self.home()

    def home(self):
        self.send_velocity(self.home_vel)
        while True:
            if self.current > self.home_current_threshold:
                self.home_pos = self.current_pos + self.home_offset
                self.send_position(self.home_pos)
                break

    def send_torque(self, torq):
        #set Torq control and passthrough
        self.bus.send(can.Message(
        arbitration_id=(self.node_id << 5 | 0x0b), 
        data=struct.pack('<II', 1,1),
        is_extended_id=False
        ))
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x0e),
            data=struct.pack('<f', torq)
        ))

    def send_velocity(self, vel):
        #Set velocity ramp control mode
        self.bus.send(can.Message(
        arbitration_id=(self.node_id << 5 | 0x0b), 
        data=struct.pack('<II', 2,2),
        is_extended_id=False
        ))

        self.bus.send(can.Message(
        arbitration_id=(self.node_id << 5 | 0x04), 
        data=struct.pack('<BHBf', 1,403,0, self.vel_ramp_rate),
        is_extended_id=False
        ))

        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', vel, 0.0), # 1.0: velocity, 0.0: torque feedforward
            is_extended_id=False
        ))
        
    def send_position(self, pos):

        #set command to passthrough and input to position
        self.bus.send(can.Message(
            abitration_id = (self.node_id << 5 | 0x0b),
            data = struct.pack('<II', 3, 1),
            is_extended_id = False
        ))
        
        self.bus.send(can.Message(
            abitration_id = (self.node_id << 5 | 0x0c),
            data = struct.pack('<fHH', pos, 0.0, 0.0),
            is_extended_id = False
        ))

    #Define a callback for watching can messages:
    def read_can(self):
        #Get a buffer of stored messages and iterate over it
        can_msgs = self.get_can_buffer()

        for can_msg in can_msgs:

            #Don't do anything if a None message gets through
            if can_msg == None:
                continue

            #Masks for getting the command and node ids
            node_mask = (1 << 6) - 1 
            cmd_mask = (1 << 5) - 1

            #First pull out and save the node ID and command ID:
            node_id = (can_msg.arbitration_id >> 5) & node_mask
            cmd_id = can_msg.arbitration_id & cmd_mask
                
            #Probably should check for the RTR bit in case someone specifically requests data while this is running
            #if can_msg.rtr: #This means the message is a request -> no data
            #	continue
            if node_id == self.node_id:
            #Use match with the command id to unpack the message correctly:
                result = {} #Assign this an empty dict in case the match falls thru
                match cmd_id:
                    case 0x01: #Heartbeat
                        pass	
                    case 0x09: #Encoder Estimate of Position/Velocity
                        pos_estimate, vel_estimate = struct.unpack('<ff', bytes(can_msg.data))
                        #self.get_logger().info(f'position: {pos_estimate}')
                        self.current_pos = pos_estimate
                    
                    case 0x14: #Q Axis motor current set/measured
                        iq_set, iq_measured = struct.unpack('<ff', bytes(can_msg.data))
                        self.current = iq_measured

                    #case 0x1c: #Torque Target/Estimate

    #Abstraction for getting all can messages currently in the buffer:
    def get_can_buffer(self):

        #Return a max of 1000 msgs, that way we don't miss publishing
        max_return_msgs = 1000

        can_msgs = []
        
        msg_count = 0

        while True:
            #Check max msgs first (that way we don't lose a msg)
            if msg_count == max_return_msgs-1:
                break
        
            #Read the msg
            can_msg = self.bus.recv(timeout=0)
        
            #We get to the last msg if reading it returns None:
            if can_msg == None:
                break

            #Add them to a list and count the number:
            #can_msgs[msg_count] = can_msg
            can_msgs.append(can_msg)
            msg_count += 1

        #Return list of msgs
        return can_msgs




def main(args=None):
    rclpy.init(args=args)

    gripper_node = GripperCanControl()

    rclpy.spin(gripper_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gripper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()