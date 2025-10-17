import pickle
import socket
import struct

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class StreamImageSubscriber(Node):
    """
    ROS 2 Node to subscribe to the /camera/color/image_raw topic
    and display the video stream using OpenCV.
    """
    ENCODING_PARAMS = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    
    def __init__(self):
        super().__init__('stream_image_subscriber')
        self.declare_parameter('streaming_host', '127.0.0.1')
        self.declare_parameter('streaming_port', 8089)
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # The topic published by the RealSense node
            self.listener_callback,
            10)
        
        # Used to convert ROS Image messages to OpenCV images
        self.br = CvBridge()
        socket_ip = self.get_parameter('streaming_host').value
        port_number = self.get_parameter('streaming_port').value
        if not isinstance(port_number, int):
            port_number = int(port_number)
        self.get_logger().info('Image subscriber node started. Waiting for images...')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((socket_ip, port_number))
        self.get_logger().info("Socket connected to {} on port {}".format(socket_ip, port_number))
        

    def listener_callback(self, data):
        """
        Callback function that processes the received Image message.
        """
        self.get_logger().debug('Receiving video frame')

        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
            _, encoded_frame = cv2.imencode(".jpg", current_frame, self.ENCODING_PARAMS)
            # Serialize the encoded array for transmission
            data = pickle.dumps(encoded_frame, 0)
            # Prepend the size of the data payload
            message_size = struct.pack(">L", len(data))
            # Send the size and then the data over the socket
            self.socket.sendall(message_size + data)
        except Exception as e:
            self.get_logger().error(f'Error converting or sending image: {e}')
            
        
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = StreamImageSubscriber()
    
    # Use a try/finally block for clean shutdown
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    
    # Destroy the node and shutdown ROS 2
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()