#import pickle
#import socket
#import struct

# from time import sleep

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import subprocess


# from ultralytics import YOLO
import onnxruntime as ort
import numpy as np


class StreamImageSubscriber(Node):
    """
    ROS 2 Node to subscribe to the /camera/color/image_raw topic
    and display the video stream using OpenCV.
    """

    ENCODING_PARAMS = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    ONNX_SESSION_OPTS = ort.SessionOptions()
    ONNX_SESSION_OPTS.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

    MODEL_CLASES = {
        0: "leopoldo"
    }

    def __init__(self):
        super().__init__("stream_image_subscriber")
        # self.declare_parameter("streaming_host", "127.0.0.1")
        # self.declare_parameter("streaming_port", 8089)
        self.declare_parameter("use_vision", 1)
        self.declare_parameter("vision_model", "models/yolo11m.pt")
        self.declare_parameter("stream_host", "0.0.0.0")
        self.declare_parameter("stream_port", "")
        self.declare_parameter("stream_path", "live/stream")
        self.declare_parameter("stream_fps", 30)
        self.declare_parameter("stream_res", "640x480")
        self.declare_parameter("stream_output_scale", "640x480")
        self.subscription = self.create_subscription(
            Image,
            "/color/image_raw",  # The topic published by the RealSense node
            self.listener_callback,
            10,
        )

        self.vision_model = None
        self.model_input_height = None
        self.model_input_width = None
        self.model_input_names = None
        if self.get_parameter("use_vision").value:
            self.get_logger().info("Inicializando modelo...")
            self.vision_model = ort.InferenceSession(
                self.get_parameter("vision_model").value,
                sess_options=self.ONNX_SESSION_OPTS,
                providers=["TensorrtExecutionProvider",
                           "CUDAExecutionProvider"],
            )
            self.model_input_width = self.vision_model.get_inputs()[0].shape[-1]
            self.model_input_height = self.vision_model.get_inputs()[0].shape[-2]
            self.model_input_names = self.vision_model.get_inputs()[0].name
            
            self.get_logger().info("Width: {} - Height: {} - Names: {}".format(self.model_input_width, self.model_input_height, self.model_input_names))
            self.get_logger().info("{}".format(self.vision_model.get_outputs()[0].name))
            self.get_logger().info("{}".format(self.vision_model.get_outputs()[1].name))
            self.get_logger().info("{}".format(self.vision_model.get_outputs()[2].name))
        
        # Used to convert ROS Image messages to OpenCV images
        self.br = CvBridge()
        stream_host = self.get_parameter("stream_host").value
        stream_port = self.get_parameter("stream_port").value
        stream_path = self.get_parameter("stream_path").value
        stream_url = "rtmp://{}{}/{}".format(stream_host,
                                              stream_port, stream_path)

        stream_res = self.get_parameter("stream_res").value
        stream_w, stream_h = stream_res.split("x")
        stream_fps = str(self.get_parameter("stream_fps").value)
        
        
        rtsp_command = [
            'ffmpeg',
            '-re',  # Read input at native frame rate
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', stream_res,
            '-r', str(stream_fps),
            '-i', '-',
            "-vf", "scale={}:{}".format(stream_w, stream_h),
            '-c:v', 'libx264',
            '-preset', 'veryfast',
            '-tune', 'zerolatency',
            '-pix_fmt', 'yuv420p',
            '-f', 'flv',
            #'-rtsp_transport', 'tcp',  # Use TCP for reliability
            stream_url
        ]
        
        #gst_command = [
        #    'gst-launch-1.0',
        #    'fdsrc', 'fd=0',
        #    '!', 'rawvideoparse', 'format=bgr24', f'width={stream_w}', f'height={stream_h}', f'framerate={stream_fps}/1',
        #    '!', 'videoconvert',
        #    '!', 'nvvidconv',
        #    '!', 'nvh264enc', 'bitrate=2000000', 'iframeinterval=15', 'insert-sps-pps=true',
        #    '!', 'h264parse', 'config-interval=1',
        #    '!', 'flvmux', 'streamable=true',
        #    '!', 'rtmpsink', f'location={stream_url}'
        #]

        # self.get_logger().info(rtsp_command)

        #self.rtsp_proto = subprocess.Popen(rtsp_command, stdin=subprocess.PIPE, bufsize=0)
        
    def __make_inference__(self, frame):
        # self.get_logger().info("Detecting objects...")
        scale_x = 1 #frame.shape[1] / self.model_input_width
        scale_y = 1 #frame.shape[0] / self.model_input_height
        rsz_frame = cv2.resize(frame, (self.model_input_height, self.model_input_width))
        rsz_frame = rsz_frame.astype(np.float32) / 255.0
        rsz_frame = np.expand_dims(rsz_frame, axis=0)
        rsz_frame = np.transpose(rsz_frame, (0, 3, 1, 2))
        #self.get_logger().info("Resize frame: {}".format(rsz_frame.shape))
        model_out = self.vision_model.run(None, {self.model_input_names: rsz_frame})
        
        #self.get_logger().info("Data: {}".format(model_out[1]))
        
        for det in model_out[1][0]:
            #self.get_logger().info("Data: {}".format(det))
            x1, y1, x2, y2 = det
            #if score < 0.75:
            #    continue
            x1 = int(x1 * scale_x)
            x2 = int(x2 * scale_x)
            y1 = int(y1 * scale_y)
            y2 = int(y2 * scale_y)
            #cls_tk = int(cls_tk)
            color = (0, 255, 0)  # green
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            #self.get_logger().info("Objeto {} detectado".format(self.MODEL_CLASES[int(cls_tk)]))
        
        
        return frame

    def listener_callback(self, data):
        """
        Callback function that processes the received Image message.
        """
        #self.get_logger().info("Receiving video frame")

        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.br.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
            #self.get_logger().info("Frame shape: {}".format(current_frame.shape))
            if self.vision_model:
                #self.get_logger().info("Making inference")
                current_frame = self.__make_inference__(current_frame)
            #self.get_logger().info("Sending frame")
            
            cv2.imshow("Camara", current_frame)
            cv2.waitKey(1)
            #self.rtsp_proto.stdin.write(current_frame.tobytes())
            #try:
            #    self.rtsp_proto.stdin.flush()
            #except Exception:
            #    pass

        except Exception as e:
            self.get_logger().error(f"Error converting or sending image: {e}")


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


if __name__ == "__main__":
    main()
