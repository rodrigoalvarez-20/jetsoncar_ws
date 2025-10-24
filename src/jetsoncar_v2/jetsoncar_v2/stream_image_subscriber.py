import pickle
import socket
import struct

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
        0: "person",
        1: "bicycle",
        2: "car",
        3: "motorcycle",
        4: "airplane",
        5: "bus",
        6: "train",
        7: "truck",
        8: "boat",
        9: "traffic light",
        10: "fire hydrant",
        11: "stop sign",
        12: "parking meter",
        13: "bench",
        14: "bird",
        15: "cat",
        16: "dog",
        17: "horse",
        18: "sheep",
        19: "cow",
        20: "elephant",
        21: "bear",
        22: "zebra",
        23: "giraffe",
        24: "backpack",
        25: "umbrella",
        26: "handbag",
        27: "tie",
        28: "suitcase",
        29: "frisbee",
        30: "skis",
        31: "snowboard",
        32: "sports ball",
        33: "kite",
        34: "baseball bat",
        35: "baseball glove",
        36: "skateboard",
        37: "surfboard",
        38: "tennis racket",
        39: "bottle",
        40: "wine glass",
        41: "cup",
        42: "fork",
        43: "knife",
        44: "spoon",
        45: "bowl",
        46: "banana",
        47: "apple",
        48: "sandwich",
        49: "orange",
        50: "brocolli",
        51: "carrot",
        52: "hot dog",
        53: "pizza",
        54: "donut",
        55: "cake",
        56: "chair",
        57: "couch",
        58: "potted plant",
        59: "bed",
        60: "dining table",
        61: "toilet",
        62: "tv",
        63: "laptop",
        64: "mouse",
        65: "remote",
        66: "keyboard",
        67: "cell phone",
        68: "microwave",
        69: "oven",
        70: "toaster",
        71: "sink",
        72: "refrigerator",
        73: "book",
        74: "clock",
        75: "vase",
        76: "scissors",
        77: "teddy bear",
        78: "hair drier",
        79: "toothbrush",
    }

    def __init__(self):
        super().__init__("stream_image_subscriber")
        # self.declare_parameter("streaming_host", "127.0.0.1")
        # self.declare_parameter("streaming_port", 8089)
        self.declare_parameter("use_yolo", 0)
        self.declare_parameter("stream_host", "0.0.0.0")
        self.declare_parameter("stream_port", 1935)
        self.declare_parameter("stream_path", "stream/detections")
        self.declare_parameter("stream_fps", 30)
        self.declare_parameter("stream_res", "640x480")
        self.declare_parameter("use_cam", 0)
        self.subscription = self.create_subscription(
            Image,
            "/camera/color/image_raw",  # The topic published by the RealSense node
            self.listener_callback,
            10,
        )

        self.yolo_model = None
        if self.get_parameter("use_yolo").value:
            self.get_logger().info("Inicializando modelo...")
            self.yolo_model = ort.InferenceSession(
                "yolo11m.onnx",
                self.ONNX_SESSION_OPTS,
                providers=["TensorrtExecutionProvider",
                           "CUDAExecutionProvider"],
            )

        # Used to convert ROS Image messages to OpenCV images
        self.br = CvBridge()
        stream_host = self.get_parameter("stream_host").value
        stream_port = self.get_parameter("stream_port").value
        stream_path = self.get_parameter("stream_path").value
        stream_url = "rtsp://{}:{}/{}".format(stream_host,
                                              stream_port, stream_path)

        stream_res = self.get_parameter("stream_res").value
        # self.stream_w = stream_res.split("x")[1]
        # self.stream_h = stream_res.split("x")[0]

        rtsp_command = [
            'ffmpeg',
            '-re',  # Read input at native frame rate
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', stream_res,
            '-r', str(self.get_parameter("stream_fps").value),
            '-i', '-',
            "-vf", "scale=640:480",
            '-c:v', 'libx264',
            '-preset', 'veryfast',
            '-tune', 'zerolatency',
            '-pix_fmt', 'yuv420p',
            '-f', 'rtsp',
            '-rtsp_transport', 'tcp',  # Use TCP for reliability
            stream_url
        ]

        # print(" ".join(rtsp_command))

        # self.get_logger().info(rtsp_command)

        self.rtsp_proto = subprocess.Popen(rtsp_command, stdin=subprocess.PIPE)

    def listener_callback(self, data):
        """
        Callback function that processes the received Image message.
        """
        self.get_logger().debug("Receiving video frame")

        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.br.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
            # self.get_logger().info(self.yolo_model)
            if self.yolo_model:
                # self.get_logger().info("Detecting objects...")
                input_name = self.yolo_model.get_inputs()[0].name
                scale_x = current_frame.shape[1] / 320
                scale_y = current_frame.shape[0] / 320
                rsz_frame = cv2.resize(current_frame, (320, 320))
                rsz_frame = rsz_frame.astype(np.float32) / 255.0
                rsz_frame = np.expand_dims(rsz_frame, axis=0)
                rsz_frame = np.transpose(rsz_frame, (0, 3, 1, 2))
                model_out = self.yolo_model.run(
                    None, {input_name: rsz_frame})[0][0]
                for det in model_out:
                    x1, y1, x2, y2, score, cls = det
                    if score < 0.75:
                        continue
                    x1 = int(x1 * scale_x)
                    x2 = int(x2 * scale_x)
                    y1 = int(y1 * scale_y)
                    y2 = int(y2 * scale_y)
                    cls = int(cls)
                    color = (0, 255, 0)  # green
                    cv2.rectangle(current_frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(
                        current_frame,
                        "Class: {}".format(self.MODEL_CLASES.get(cls)),
                        (x1, max(0, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        2,
                    )
                # self.get_logger().info(model_out)

            self.rtsp_proto.stdin.write(current_frame.tobytes())

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
