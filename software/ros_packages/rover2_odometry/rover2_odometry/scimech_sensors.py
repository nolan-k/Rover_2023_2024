import rclpy
import rclpy.logging
from rclpy.node import Node
import serial
from std_msgs.msg import Float32MultiArray
from time import time, sleep

class ScimechSensors(Node):

    def __init__(self):

        super().__init__('scimech_sensors')

        self.scimech_data_publisher_ = self.create_publisher(Float32MultiArray, 'scimech/data', 10)
        
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)        

        self.logger = self.get_logger()

        self.humidity = 0
        self.hydrogen = 0
        self.ozone = 0
        self.temperature = 0

        self.scimech_arduino = serial.Serial('PLACEHOLDER',115200, timeout=1)
                 

    def read_scimech_serial_data(self):
        
        if self.scimech_arduino > 0:
            line = self.scimech_arduino.readline().decode('utf-8').rstrip()
            self.logger.info("Reading from arduino: {}".format(line))
            data_array = line.split(",")
            
            self.temperature = data_array[0]
            self.humidity = data_array[1]
            self.hydrogen = data_array[2]
            self.ozone = data_array[3]
        else:
            self.logger.error("Arduino serial device not found")
            
                

    def timer_callback(self):


        msg = Float32MultiArray()
        msg.data = [self.humidity,self.hydrogen,self.ozone,self.temperature]
        self.publisher_.publish(msg)
        self.logger.info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    scimech_sensors = ScimechSensors()

    rclpy.spin(scimech_sensors)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scimech_sensors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()