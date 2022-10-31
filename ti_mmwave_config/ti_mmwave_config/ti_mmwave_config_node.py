import serial
import rclpy
from rclpy.node import Node
from ti_mmwave_config_interface.srv import SendConfig, StartSensor, StopSensor

class ti_mmwave_config(Node):
    def __init__(self) -> None:
        super().__init__('ti_mmwave_config_node')

        # create services
        self.config_srv = self.create_service(SendConfig, "send_config",
                                                            self.config_cb)
        self.start_srv = self.create_service(StartSensor, "start_sensor",
                                                        self.start_sensor_cb)
        self.stop_srv = self.create_service(StopSensor, "stop_sensor", 
                                                        self.stop_sensor_cb)
        
        # read parameters
        self.declare_parameter('config_port', 'None')
        self.declare_parameter('config_file', 'None')
        self.config_port = self.get_parameter(
                    'config_port').get_parameter_value().string_value
        self.config_file = self.get_parameter(
                    'config_file').get_parameter_value().string_value
        
        # serial port to config the sensor
        self.serial_config = None
        
        # list of valid responses from the sensor
        self.valid_responses = set(["Done",
                                    "Ignored: Sensor is already stopped",
                                    "Ignored: Sensor is already started"])

    def _connect_config_port(self, baud_rate=115200, timeout=1):
        """
        connect to the config port.
        """
        try:
            self.get_logger().info("Connecting to config port...")
            self.serial_config = serial.Serial(
                    self.config_port, baud_rate, timeout=timeout)
            self.get_logger().info("Connection to sensor established.")
            return True
        except serial.SerialException as e:
            self.get_logger().error("{}".format(e))
            return False
        except FileNotFoundError:
            self.get_logger().error(
                    f"{self.config_port} is an invalid serial port.")
            return False
        except ValueError:
            self.get_logger().error("Baud rate is invalid.")
            return False

    def config_cb(self, request, response):
        """
        send configuration file to the sensor
        """
        if request.data:
            if not self._connect_config_port():
                response.success = False
                return response
            else:
                cfg_commands = []
                # read the config commands into a list
                with open(self.config_file) as cfg_file:
                    for line in cfg_file:
                        if line[0] == '%' or line[0] == '\n':
                            continue
                        else:
                            cfg_commands.append(line)
                self.get_logger().info('Sending config to the sensor...')
                for command in cfg_commands:
                    self.get_logger().debug('Sending: {}'.format(command))
                    self.serial_config.write(command.encode())
                    ret = self.serial_config.readline()
                    ret = ret.decode('utf-8')[:-1].strip()
                    self.get_logger().debug(ret)
                    ret_status = self.serial_config.readline()
                    ret_status = ret_status.decode('utf-8')[:-1].strip()
                    self.get_logger().debug(ret_status)
                    # flush the buffer
                    self.serial_config.flushOutput()
                    self.serial_config.flushInput()
                    if (ret_status not in self.valid_responses and
                                                ret not in self.valid_responses):
                        self.get_logger().error(
                                "Failed to send command: {}".format(command))
                        response.success = False
                        return response
                self.get_logger().info(
                        'Config sent to the sensor successfuly.')
                response.success = True
                return response

    def start_sensor_cb(self, request, response):
        """
        start the data stream from the sensor
        """
        if request.data:
            self.get_logger().info('Starting the sensor...')
            if self.serial_config is None:
                self.get_logger().error(
                    'Serial port has not been initialized.')
                response.success = False
            else:
                self.serial_config.write("sensorStart 0\n".encode())
                ret = self.serial_config.readline()
                ret = ret.decode('utf-8')[:-1].strip()
                self.get_logger().debug(ret)
                ret_status = self.serial_config.readline()
                ret_status = ret_status.decode('utf-8')[:-1].strip()
                self.get_logger().debug(ret_status)
                # flush the buffer
                self.serial_config.flushOutput()
                self.serial_config.flushInput()
                if (ret_status not in self.valid_responses 
                                        and ret not in self.valid_responses):
                    self.get_logger().error('Failed to start the sensor.')
                    response.success = False
                else:
                    response.success = True
        return response
            

    def stop_sensor_cb(self, request, response):
        """
        stop the data stream from the sensor
        """
        if request.data:
            self.get_logger().info('Stopping the sensor...')
            if self.serial_config is None:
                self.get_logger().error(
                    'Serial port has not been initialized.')
                response.success = False
            else:
                self.serial_config.write("sensorStop\n".encode())
                ret = self.serial_config.readline()
                ret = ret.decode('utf-8')[:-1].strip()
                self.get_logger().debug(ret)
                ret_status = self.serial_config.readline()
                ret_status = ret_status.decode('utf-8')[:-1].strip()
                self.get_logger().debug(ret_status)
                # flush the buffer
                self.serial_config.flushOutput()
                self.serial_config.flushInput()
                if (ret_status not in self.valid_responses 
                                        and ret not in self.valid_responses):
                    self.get_logger().error('Failed to stop the sensor.')
                    response.success = False
                else:
                    response.success = True
        return response


def main():
    rclpy.init()
    node = ti_mmwave_config()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
