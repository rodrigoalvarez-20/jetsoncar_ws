import rclpy
from rclpy.node import Node
from time import sleep
import cv2
import subprocess
import onnxruntime as ort
import numpy as np


class StreamCameraSubscriber(Node):

    ONNX_SESSION_OPTS = ort.SessionOptions()
    ONNX_SESSION_OPTS.graph_optimization_level = (
        ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    )

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
        super().__init__("stream_camera_subscriber")
        # self.declare_parameter("streaming_host", "127.0.0.1")
        # self.declare_parameter("streaming_port", 8089)
        self.declare_parameter("use_yolo", 0)
        self.declare_parameter("yolo_model", "models/yolo11m.pt")
        self.declare_parameter("stream_host", "0.0.0.0")
        self.declare_parameter("stream_port", "")
        self.declare_parameter("stream_path", "live/stream")
        self.declare_parameter("stream_fps", 30)
        self.declare_parameter("stream_res", "640x480")
        self.declare_parameter("stream_output_scale", "640x480")

        local_camera = cv2.VideoCapture(0)

        while not local_camera.isOpened():
            self.get_logger().error("Error al conectar con la camara...")
            self.get_logger().info("Reintentando conexion")
            local_camera = cv2.VideoCapture(0)
            sleep(3)

        # Request resolution
        req_width, req_height = map(
            int, self.get_parameter("stream_res").value.split("x")
        )
        local_camera.set(cv2.CAP_PROP_FRAME_WIDTH, req_width)
        local_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, req_height)

        # Get actual resolution from camera
        camera_width = int(local_camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        camera_height = int(local_camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Actual camera resolution: {camera_width}x{camera_height}")

        self.local_camera = local_camera

        self.yolo_model = None
        self.model_input_height = None
        self.model_input_width = None
        self.model_input_names = None
        if self.get_parameter("use_yolo").value:
            self.get_logger().info("Inicializando modelo...")
            self.yolo_model = ort.InferenceSession(
                self.get_parameter("yolo_model").value,
                sess_options=self.ONNX_SESSION_OPTS,
                providers=["TensorrtExecutionProvider", "CUDAExecutionProvider"],
            )
            self.model_input_width = self.yolo_model.get_inputs()[0].shape[-1]
            self.model_input_height = self.yolo_model.get_inputs()[0].shape[-2]
            self.model_input_names = self.yolo_model.get_inputs()[0].name

        stream_host = self.get_parameter("stream_host").value
        stream_port = self.get_parameter("stream_port").value
        stream_path = self.get_parameter("stream_path").value
        stream_url = "rtmp://{}{}/{}".format(stream_host, stream_port, stream_path)

        stream_res = self.get_parameter("stream_res").value
        stream_w, stream_h = stream_res.split("x")
        stream_fps = str(self.get_parameter("stream_fps").value)

        self.get_logger().info("Streaming to: {}".format(stream_url))
        ffmpeg_cmd = [
            "ffmpeg",
            "-re",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", "{}x{}".format(camera_width, camera_height),
            "-r", str(stream_fps),
            "-i", "-",                     # read frames from stdin
            "-vf", "scale={}".format(str(self.get_parameter("stream_output_scale").value.replace("x", ":"))),
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            #"-profile:v", "baseline",
            "-g", "15",                    # Short GOP = lower latency
            "-keyint_min", "15",
            "-bf", "0",                    # No B-frames
            "-x264-params", "keyint=30:min-keyint=30:no-scenecut=1",
            "-f", "flv",
            stream_url
        ]
        
        #self.rtsp_proto = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

        self.__stream_data__()

    def __make_inference__(self, frame):
        # self.get_logger().info("Detecting objects...")
        scale_x = frame.shape[1] / self.model_input_width
        scale_y = frame.shape[0] / self.model_input_height
        rsz_frame = cv2.resize(frame, (self.model_input_height, self.model_input_width))
        rsz_frame = rsz_frame.astype(np.float32) / 255.0
        rsz_frame = np.expand_dims(rsz_frame, axis=0)
        rsz_frame = np.transpose(rsz_frame, (0, 3, 1, 2))
        model_out = self.yolo_model.run(None, {self.model_input_names: rsz_frame})[0][0]
        for det in model_out:
            x1, y1, x2, y2, score, cls_tk = det
            if score < 0.75:
                continue
            x1 = int(x1 * scale_x)
            x2 = int(x2 * scale_x)
            y1 = int(y1 * scale_y)
            y2 = int(y2 * scale_y)
            cls_tk = int(cls_tk)
            color = (0, 255, 0)  # green
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
            cv2.putText(
                frame,
                "Class: {}".format(self.MODEL_CLASES.get(cls_tk)),
                (x2 - 10, max(0, y2 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
            )
        return frame

    def __stream_data__(self):

        while True:
            ret, current_frame = self.local_camera.read()

            if self.yolo_model:
                current_frame = self.__make_inference__(current_frame)

            if ret:  # If frame not read correctly, break the loop
                #self.rtsp_proto.stdin.write(current_frame.tobytes())
                cv2.imshow("Camara", current_frame)
            else:
                self.get_logger().warning("No frame end. Skipping...")


def main(args=None):
    rclpy.init(args=args)
    local_cam_subs = StreamCameraSubscriber()

    # Use a try/finally block for clean shutdown
    try:
        rclpy.spin(local_cam_subs)
        sleep(0.01)
    except KeyboardInterrupt:
        if local_cam_subs.local_camera:
            local_cam_subs.local_camera.release()
        cv2.destroyAllWindows()
    except Exception as ex:
        if local_cam_subs.local_camera:
            local_cam_subs.local_camera.release()
        cv2.destroyAllWindows()

    # Destroy the node and shutdown ROS 2
    rclpy.shutdown()


if __name__ == "__main__":
    main()
