import struct
import can
import sys

bus = can.interface.Bus("can0", interface="socketcan")

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

vel = sys.argv[1]
vel = float(vel)
vel_ramp = 2

nodes = [0,1,2,3,4,5]
for node_id in nodes:
    print("Setting {} to trap velocity: {}".format(node_id,vel))

    # Put axis into closed loop control state
#    bus.send(can.Message(
#    arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
#    data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
#    is_extended_id=False
#    ))
    #Set velocity ramp control mode
    bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x0b), 
    data=struct.pack('<II', 2,2),
    is_extended_id=False
    ))
    #Set velocity ramp rate
#    bus.send(can.Message(
#    arbitration_id=(node_id << 5 | 0x04), 
#    data=struct.pack('<BHBf', 1,413,0,vel_ramp),
#    is_extended_id=False
#    ))
    #Send a test velocity
    

for node_id in nodes:

    bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
    data=struct.pack('<ff', vel, 0.0), # 1.0: velocity, 0.0: torque feedforward
    is_extended_id=False
    ))
    
