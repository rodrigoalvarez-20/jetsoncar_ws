import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSubscriber(Node):
    """
    ROS 2 Node to subscribe to the /camera/color/image_raw topic
    and display the video stream using OpenCV.
    """
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # The topic published by the RealSense node
            self.listener_callback,
            10)
        
        # Used to convert ROS Image messages to OpenCV images
        self.br = CvBridge()
        self.get_logger().info('Image subscriber node started. Waiting for images...')

    def listener_callback(self, data):
        """
        Callback function that processes the received Image message.
        """
        self.get_logger().debug('Receiving video frame')

        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
            
            # Display the image in a window
            cv2.imshow("SR300 Color Stream", current_frame)
            cv2.waitKey(1)  # Required to update the OpenCV window
        
        except Exception as e:
            self.get_logger().error(f'Error converting or displaying image: {e}')
        
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    
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