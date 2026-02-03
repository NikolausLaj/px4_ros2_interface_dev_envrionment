import rclpy
from rclpy.node import Node

from pieps_interfacer.msg import PiepsMeasurements

# TODO Create Launch file
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('pieps_node')

        _serialPort, _baudrate, _timeout = self.getParameters()

        self.publisher_ = self.create_publisher(PiepsMeasurements, 'topic', 10) 


    def getParameters(self):
        self.declare_parameter('SERIAL_PORT', '/dev/ttyUSB0')
        self.declare_parameter('BAUDRATE', 115200)
        self.declare_parameter('TIMEOUT', 1)

        serial_port = self.get_parameter('SERIAL_PORT').get_parameter_value().string_value
        baudrate = self.get_parameter('BAUDRATE').get_parameter_value().integer_value
        timeout = self.get_parameter('TIMEOUT').get_parameter_value().integer_value

        return serial_port, baudrate, timeout

    def setupSerial(self, serial_port, baudrate, timeout):
        pass

    def readPiepsData(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()