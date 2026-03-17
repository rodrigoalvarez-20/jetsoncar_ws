import rclpy
from rclpy.node import Node
import cv2
from time import sleep
import serial
import socket
import selectors
import types

SOCKET_HOST = '0.0.0.0' # Listen on all available interfaces
SOCKET_PORT = 5000

class RCCarWheelSocket(Node):

    def __init__(self):
        super().__init__('rc_car_wheel_socket')
        # Puente entre carro y Jetson (Arduino o ESP32)
        self.declare_parameter('bridge_port', "/dev/ttyACM0")
        self.declare_parameter('bridge_baudrate', 115200)
        
        
        self.rc_bridge = serial.Serial(self.get_parameter("bridge_port").value, int(
            self.get_parameter("bridge_baudrate").value), timeout=5)
        
        
        self._steering_value = 90
        self._throttle_value = 90
        self.__claxon = 0
        self._is_break_active = False
        self._is_reverse_active = False
        self.__test_bridge__()
        
        # --- Non-Blocking Socket Setup (NEW) ---
        self.sel = selectors.DefaultSelector()
        self.listen_socket = self.__setup_socket_listener__()
        
        # --- ROS 2 Timer for Socket Polling (NEW) ---
        # Create a timer that calls the socket handler 50 times a second (20ms interval)
        timer_period = 0.02 
        self.socket_timer = self.create_timer(timer_period, self.socket_handler)
        
        self.get_logger().info(f"Socket Listening on {SOCKET_HOST}:{SOCKET_PORT}")
        self.get_logger().info("Listo para conducir.")

    @property
    def steering_value(self):
        return self._steering_value

    @steering_value.setter
    def steering_value(self, steering_value):
        # print(self.steering_value, steering_value)

        if self._steering_value != steering_value:
            self._steering_value = steering_value
            # self.get_logger().info("New Steering Value... | {}".format(steering_value))
            self.__send_controls__(steering_value, self.throttle_value)
            # Enviar nuevo valor al arduino

    @property
    def throttle_value(self):
        return self._throttle_value

    @throttle_value.setter
    def throttle_value(self, throttle_value):
        if self._throttle_value != throttle_value:
            self._throttle_value = throttle_value
            # Enviar nuevo valor al arduino
            self.__send_controls__(self.steering_value, throttle_value)
    
    
    @property
    def claxon(self):
        return self.__claxon

    @claxon.setter
    def claxon(self, claxon_value):
        if self.__claxon != claxon_value:
            self.__claxon = claxon_value
            # self.get_logger().info("New Steering Value... | {}".format(steering_value))
            self.__send_claxon__()
            # Enviar nuevo valor al arduino

    def __setup_socket_listener__(self):
        """Sets up and registers the non-blocking listening TCP socket."""
        lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        lsock.bind((SOCKET_HOST, SOCKET_PORT))
        lsock.listen()
        
        # Set the socket to non-blocking mode
        lsock.setblocking(False)
        
        # Register the listening socket with the selector for READ events
        self.sel.register(lsock, selectors.EVENT_READ, data=None)
        
        return lsock

    def __accept_wrapper__(self, sock):
        """Accepts new connections and sets them up."""
        try:
            conn, addr = sock.accept()
            self.get_logger().info(f"Accepted connection from {addr}")
            conn.setblocking(False)
            
            # Simple data structure to hold buffer for each client
            # 'outb' is not strictly needed for just listening, but good practice
            data = types.SimpleNamespace(addr=addr, inb=b'', outb=b'')
            
            # Register for READ events on the client connection
            self.sel.register(conn, selectors.EVENT_READ, data=data)
        except BlockingIOError:
            # Should not happen since select indicated readiness
            pass
        except Exception as e:
            self.get_logger().error(f"Error accepting connection: {e}")
    
    def __service_connection__(self, key, mask):
        """Handles I/O for an established client connection."""
        sock = key.fileobj
        data = key.data
        
        if mask & selectors.EVENT_READ:
            try:
                # Use a small buffer size appropriate for control commands
                recv_data = sock.recv(1024) 
                
                if recv_data:
                    # Append received data to the buffer
                    data.inb += recv_data
                    
                    # Process complete messages from the buffer
                    self.__process_received_data__(data)

                else:
                    # No data means client closed connection
                    self.get_logger().info(f"Closing connection to {data.addr}")
                    self.sel.unregister(sock)
                    sock.close()
            
            except ConnectionResetError:
                self.get_logger().warning(f"Connection reset by {data.addr}")
                self.sel.unregister(sock)
                sock.close()
            except BlockingIOError:
                # Should not occur, but caught just in case
                pass
    
    def __process_received_data__(self, data):
        """
        Parses the received data buffer to extract complete commands.
        Assuming commands are line-delimited (e.g., "S:90,T:90\n")
        """
        # Look for the newline character which terminates a command
        while b'\n' in data.inb:
            command, data.inb = data.inb.split(b'\n', 1)
            
            try:
                cmd_str = command.decode().strip()
                # Example command: "S:90,T:120"
                
                parts = cmd_str.split(',')
                new_steering = float(parts[0]) #self._steering_value
                new_throttle = float(parts[1]) #self._throttle_value
                reverse = round(float(parts[2]),2) #self._throttle_value
                claxon = int(parts[3])
                self.claxon = claxon

                #for part in parts:
                #    if part.startswith('S:'):
                #        new_steering = int(part[2:])
                #    elif part.startswith('T:'):
                #        new_throttle = int(part[2:])
                        
                # Update properties, which triggers the serial send (__send_controls__)
                #self.steering_value = new_steering
                #self.throttle_value = new_throttle
                
                self.get_logger().info(f"Data rcv: {new_steering},{new_throttle},{reverse},{claxon}")
                
                self.steering_value = self.rescale_input(new_steering) + 90
                
                if reverse <= 0:
                    self.throttle_value = 90 - self.rescale_input(new_throttle, input_min_value=0, rescale_min_value=0, rescale_max_value=20)
                else:
                    self.throttle_value = 90 + self.rescale_input(new_throttle, input_min_value=0, rescale_min_value=0, rescale_max_value=20)
                

            except ValueError:
                self.get_logger().error(f"Invalid command format received: {command}")
            except Exception as e:
                self.get_logger().error(f"Error processing command: {e}")
    
    def socket_handler(self):
        """
        The main non-blocking socket loop run by the ROS 2 timer.
        """
        # Get ready sockets without blocking (timeout=0)
        events = self.sel.select(timeout=0)
        
        for key, mask in events:
            if key.data is None:
                # Listening socket is ready (new connection)
                self.__accept_wrapper__(key.fileobj)
            else:
                # Client socket is ready (read/write activity)
                self.__service_connection__(key, mask)
    
    def rescale_input(self, input_value, input_min_value=-1, input_max_value=1, rescale_min_value=-45, rescale_max_value=45):
        """
        Funcion de transformacion para convertir los valores obtenidos del 
        joystick a valores de angulo necesarios para el servomotor de direccion.
        Valores de entrada: -1 a 1
        Valores de salida: -45 a 45
        """
        return (input_value - input_min_value) * (rescale_max_value - rescale_min_value) / (input_max_value - input_min_value) + rescale_min_value

    def __test_bridge__(self):
        test_angles = [80, 135, 105, 80, 75, 45, 80]
        # Probamos direccion
        for angle in test_angles:
            self.__send_controls__(angle, 90)
            sleep(0.5)

    def __send_controls__(self, steering, throttle):
        """
        Send steering and throttle values as comma-separated string
        Example: "120,95\n"
        """
        cmd = "{},{}\n".format(steering, throttle)
        if self.rc_bridge and self.rc_bridge.isOpen():
            #self.get_logger().info("Enviando datos: {}".format(cmd))
            self.rc_bridge.write(cmd.encode())
            #sleep(0.01)
        else:
            self.get_logger().warning("No Bridge connection to send command. Skipping...")
            
    def __send_claxon__(self):
        if self.rc_bridge and self.rc_bridge.isOpen():
            cmd = "B:{}\n".format(self.claxon)
            #self.get_logger().info("Enviando datos: {}".format(cmd))
            self.rc_bridge.write(cmd.encode())
            #sleep(0.01)
        else:
            self.get_logger().warning("No Bridge connection to send command. Skipping...")

    def disconnect_from_bridge_and_socket(self):
        """Cleanly closes serial and socket connections."""
        try:
            self.rc_bridge.close()
            self.get_logger().info("Serial bridge disconnected.")
        except Exception as ex:
            self.get_logger().warning(f"Error disconnecting bridge: {ex}")
            
        try:
            self.sel.close()
            self.listen_socket.close()
            self.get_logger().info("Socket server closed.")
        except Exception as ex:
            self.get_logger().warning(f"Error closing socket server: {ex}")

    def disconnect_from_bridge(self):
        try:
            self.rc_bridge.close()
        except Exception as ex:
            self.get_logger().warning(
                "Ha ocurrido un error al comunicarse con el puente | {}".format(ex))


def main(args=None):
    rclpy.init(args=args)
    rc_car_subscriber = RCCarWheelSocket()

    # Use a try/finally block for clean shutdown
    try:
        rclpy.spin(rc_car_subscriber)
        sleep(0.01)
    except KeyboardInterrupt:
        if rc_car_subscriber.device:
            rc_car_subscriber.device.deactivate()
        if rc_car_subscriber.rc_bridge:
            rc_car_subscriber.disconnect_from_bridge_and_socket()
            #rc_car_subscriber.disconnect_from_bridge()
    except Exception as ex:
        pass

    # Destroy the node and shutdown ROS 2
    rclpy.shutdown()


if __name__ == '__main__':
    main()
