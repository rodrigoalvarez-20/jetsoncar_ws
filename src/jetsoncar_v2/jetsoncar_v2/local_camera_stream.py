# import pickle
# import socket
# import struct

# from time import sleep

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import supervision as sv
from .mlp_model.lfy import RCCarHead
import pyrealsense2.pyrealsense2 as rs
from time import sleep
# import torchvision.ops as ops

# from ultralytics import YOLO
import onnxruntime as ort
from transformers import YolosImageProcessor
import numpy as np
import torch
from transformers.models.yolos.modeling_yolos import YolosObjectDetectionOutput


ort.set_default_logger_severity(3)

class LocalCameraStream(Node):

    ENCODING_PARAMS = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    ONNX_SESSION_OPTS = ort.SessionOptions()
    ONNX_SESSION_OPTS.graph_optimization_level = (
        ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    )

    LABEL2ID = {0: "leopoldo"}

    ID2LABEL = {"leopoldo": 0}

    def __init__(self):
        super().__init__("local_camera_stream")
        # self.declare_parameter("streaming_host", "127.0.0.1")
        # self.declare_parameter("streaming_port", 8089)
        self.declare_parameter("use_vision", 1)
        self.declare_parameter("use_navigation", 1)
        self.declare_parameter(
            "vision_model", "models/yolos_vanilla_export_Canopus_2_320_17.onnx"
        )
        self.declare_parameter("vision_processor", "models/Canopus_V2")
        self.declare_parameter(
            "navigation_model", "models/rc_car/Canopus_V2/weights.pt"
        )
        
        self.declare_parameter("display_camera", 1)
        
        self.camera_pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.camera_pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        #playback = profile.get_device().as_playback()
        
        device = pipeline_profile.get_device()#.as_playback()
        #device.set_real_time(False)
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        
        self.get_logger().info("Device found: {}".format(device_product_line))
        
        config.enable_stream(rs.stream.color, 320, 180, rs.format.bgr8, 30)
        self.camera_pipeline.start(config)

        self.vision_model = None
        self.vision_processor = None
        self.model_input_height = None
        self.model_input_width = None
        self.model_input_names = None
        self.navigation_model = None
        self.publisher_ = None
        if self.get_parameter("use_vision").value:
            self.get_logger().info("Inicializando modelo de vision...")
            self.vision_processor = YolosImageProcessor.from_pretrained(
                self.get_parameter("vision_processor").value
            )
            self.vision_model = ort.InferenceSession(
                self.get_parameter("vision_model").value,
                sess_options=self.ONNX_SESSION_OPTS,
                providers=["TensorrtExecutionProvider", "CUDAExecutionProvider"],
            )
            self.model_input_width = self.vision_model.get_inputs()[0].shape[-1]
            self.model_input_height = self.vision_model.get_inputs()[0].shape[-2]
            self.model_input_names = self.vision_model.get_inputs()[0].name
        
        if self.get_parameter("use_navigation").value:
            self.get_logger().info("Inicializando modelo de navegacion")
            self.navigation_model = RCCarHead.load_checkpoint(self.get_parameter("navigation_model").value)
            self.navigation_model.eval()
            self.publisher_ = self.create_publisher(Float32MultiArray, '/car/navigation', 10)
        
        
        self.poll_frames()

    def __make_inference__(self, frame):

        # self.get_logger().info("Original frame size: {}".format(frame.shape))
        # H W C
        work_frame = frame.copy()

        work_frame = cv2.resize(frame, (320, 320))
        work_frame = work_frame.astype(np.float32) / 255.0

        work_frame = np.expand_dims(work_frame, axis=0)
        # B H W C
        work_frame = np.transpose(work_frame, (0, 3, 1, 2))
        # B C H W
        logits, pred_boxes, last_hidden = self.vision_model.run(
            None, {self.model_input_names: work_frame}
        )

        yolos_out = YolosObjectDetectionOutput(
            None,
            None,
            torch.tensor(logits, device="cuda"),
            torch.tensor(pred_boxes, device="cuda"),
            last_hidden_state=torch.tensor(last_hidden, device="cuda"),
        )

        results = self.vision_processor.post_process_object_detection(
            outputs=yolos_out,
            threshold=0.7,
            target_sizes=[frame.shape[:2]],
        )[0]

        # self.get_logger().info("results: {}".format(results))

        annotated_frame = frame.copy()

        detections = sv.Detections.from_transformers(
            transformers_results=results
        ).with_nms(threshold=0.1)

        is_leopoldo_detected = False
        
        for det, conf, class_id in zip(
            detections.xyxy, detections.confidence, detections.class_id
        ):
            if conf < 0.6:
                continue
            x1, y1, x2, y2 = map(int, det)
            color = (0, 255, 0)  # green
            if class_id == self.ID2LABEL["leopoldo"]:
                is_leopoldo_detected = True
            else: is_leopoldo_detected = False
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                annotated_frame,
                "Class: {} - Conf: {}".format(self.LABEL2ID.get(class_id), conf),
                (x1 , y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
        
        #self.get_logger().info("Yolos out: {}".format(yolos_out))
        
        if is_leopoldo_detected and len(detections.xyxy) and self.navigation_model and yolos_out.last_hidden_state is not None:
            # Objeto, generar control y enviar
            
            #rc_outputs = rc_car_model.forward(outputs.last_hidden_state)[:,:,0]
            rc_outputs = self.navigation_model.forward(yolos_out.last_hidden_state)[:,:,0]
            
            rc_outputs = rc_outputs[0].cpu().detach().numpy() if len(rc_outputs) > 0 else []
            self.get_logger().info("RC Car controls: {}".format(rc_outputs))
            msg = Float32MultiArray()
            msg.data = rc_outputs.tolist()
            self.publisher_.publish(msg)
        
        del work_frame

        return annotated_frame

    def poll_frames(self):
        """
        Callback function that processes the received Image message.
        """
        while True:
            try:
                #self.get_logger().info("Polling frame")
                # Convert ROS Image message to OpenCV image
                frames = self.camera_pipeline.poll_for_frames()
                if not frames:
                    continue
                color_frame = frames.get_color_frame()
                #self.get_logger().info("Frames: {}".format(color_frame))
                if not color_frame:
                    continue
                    # Convert images to numpy arrays
                current_frame = np.asanyarray(color_frame.get_data())            
                #current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
                #self.get_logger().info("Frame: {}".format(current_frame))
                if self.vision_model:
                    current_frame = self.__make_inference__(current_frame)        
                cv2.imshow("Camara", current_frame)
                cv2.waitKey(1)
                sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"Error converting or sending image: {e}")


def main(args=None):
    rclpy.init(args=args)
    camera_stream = LocalCameraStream()

    # Use a try/finally block for clean shutdown
    try:
        #while True:
        rclpy.spin(camera_stream)
    except KeyboardInterrupt:
        pass

    # Destroy the node and shutdown ROS 2
    camera_stream.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
