import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from time import sleep
from dualsense_controller import DualSenseController
import serial
from uuid import uuid4
import os
from queue import Queue

class RCCarVanilla(Node):

    def __init__(self):
        super().__init__('rc_car_vanilla')
        self.declare_parameter('left_stick_drift', 0.08)
        self.declare_parameter('right_stick_drift', 0.0)
        self.declare_parameter('left_trigger_drift', 0.0)
        self.declare_parameter('right_trigger_drift', 0.0)
        # Puente entre carro y Jetson (Arduino o ESP32)
        self.declare_parameter('bridge_port', "/dev/ttyACM0")
        self.declare_parameter('bridge_baudrate', 115200)
        self.declare_parameter("load_camera", 0)
        self.declare_parameter("recording_path", "/home/jetson/jetsoncar_ws/recordings")
        self.device = None
        self.rc_bridge = serial.Serial(self.get_parameter("bridge_port").value, int(
            self.get_parameter("bridge_baudrate").value), timeout=5)
        
        self.camera_sub = None
        self.camera_bridge = None
        self.is_recording = False
        self.frame_count = 0
        self.record_queue = Queue()
        
        if self.get_parameter("load_camera").value:
            self.camera_sub = self.create_subscription(
                Image,
                "/color/image_raw",  # The topic published by the RealSense node
                self.listener_callback,
                10,
            )
            
            self.camera_bridge = CvBridge()
        
        self.recording_dir = self.get_parameter("recording_path").value
        self.execution_id = uuid4().hex[:12]
        self.records_path = os.path.join(self.recording_dir, self.execution_id)
        self.records_path_images = os.path.join(self.records_path, "images")
        os.makedirs(self.recording_dir, exist_ok=True)
        os.makedirs(self.records_path, exist_ok=True)
        os.makedirs(self.records_path_images, exist_ok=True)
        

        sleep(5)
        
        self._steering_value = 90
        self._throttle_value = 90
        self.__claxon = 0
        self._is_break_active = False
        self._is_reverse_active = False
        self.left_stick_drift = float(
            self.get_parameter('left_stick_drift').value)
        self.right_stick_drift = float(
            self.get_parameter('right_stick_drift').value)
        self.left_trigger_drift = float(
            self.get_parameter('left_trigger_drift').value)
        self.right_trigger_drift = float(
            self.get_parameter('right_trigger_drift').value)
        self.get_logger().info("Inicializando conexión con dispositivo...")
        self.connect_to_device()
        self.device.activate()
        self.__test_bridge__()
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

    def listener_callback(self, data):
        """
        Callback function that processes the received Image message.
        """
        #self.get_logger().info("Receiving video frame")

        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.camera_bridge.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
            #print(current_frame)
            if (self.frame_count % 15 == 0):
                if self.is_recording:
                    self.get_logger().info("Saving data to queue")
                    self.record_queue.put((current_frame, self.steering_value, self.throttle_value))
                cv2.imshow("Camara", current_frame)
                cv2.waitKey(1)
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f"Error converting or sending image: {e}")
    
    def connect_to_device(self):
        while not self.device:
            self.get_logger().info("Buscando dispositivos...")
            device_infos = DualSenseController.enumerate_devices()
            self.get_logger().info("Se han encontrado {} dispositivos".format(len(device_infos)))
            if len(device_infos) < 1:
                self.get_logger().info("Esperando conexión con dispositivo...")
                sleep(5)
                continue

            controller = DualSenseController()
            controller.left_trigger.on_change(self.on_left_trigger)
            controller.left_stick_x.on_change(self.on_left_stick_x_changed)
            controller.right_trigger.on_change(self.on_right_trigger)
            controller.btn_circle.on_down(self.on_circle_press)
            controller.btn_circle.on_up(self.on_circle_release)
            controller.btn_l1.on_down(self.on_l1_button_press)
            controller.btn_r1.on_down(self.on_r1_button_press)
            # register the error callback
            controller.on_error(self.on_error)
            controller.lightbar.set_color_red()
            self.device = controller

    def rescale_input(self, input_value, input_min_value=-1, input_max_value=1, rescale_min_value=-45, rescale_max_value=45):
        """
        Funcion de transformacion para convertir los valores obtenidos del 
        joystick a valores de angulo necesarios para el servomotor de direccion.
        Valores de entrada: -1 a 1
        Valores de salida: -45 a 45
        """
        return (input_value - input_min_value) * (rescale_max_value - rescale_min_value) / (input_max_value - input_min_value) + rescale_min_value

    def stop(self):
        pass
    
    def on_l1_button_press(self):
        self.is_recording = not self.is_recording
        self.get_logger().info("Recording is {}".format(self.is_recording))

    def on_r1_button_press(self):
        self.is_recording = False
        txt_contents = ""
        self.get_logger().info("Exporting {} items in queue".format(self.record_queue.qsize()))
        self.get_logger().info("Queue Status: {}".format(self.record_queue.empty()))
        while not self.record_queue.empty():
            #self.get_logger().info("Exporting element...")
            record_name = uuid4().hex
            qitem = self.record_queue.get()
            #print(qitem)
            img_name = os.path.join(self.records_path_images, record_name) + ".jpg"
            file_line = "{},{},{}\n".format(img_name, qitem[1], qitem[2])
            img_saved = cv2.imwrite(img_name, qitem[0], [cv2.IMWRITE_JPEG_QUALITY, 90])
            #print(img_name)
            #print(file_line)
            #print(img_saved)
            if img_saved:
                txt_contents += file_line
        csv_file = os.path.join(self.records_path, record_name) + ".csv"
        with open(csv_file, "w", encoding="utf-8") as f:
            f.write(txt_contents)
        self.get_logger().info("Export finished")

    
    def on_circle_press(self):
        self.claxon = "1"
        self.device.lightbar.set_color_green()
        
    def on_circle_release(self):
        self.claxon = "0"
        self.device.lightbar.set_color_red()

    def on_left_trigger(self, value):
        self.get_logger().debug("left trigger changed: {}".format(value))
        # El trigger izquierdo funcionará como un freno y activador de reversa
        # 0 <= value < 1.0 --> Freno
        # value = 1 --> Indicador de reversa para gatillo R2
        if value >= 0.85:
            self._is_reverse_active = True
        elif value < 0.85 and self._is_reverse_active:
            self._is_reverse_active = False
            
    def on_right_trigger(self, value):
        value_no_drift = value - self.right_trigger_drift
        
        if self._is_reverse_active:
            self.get_logger().debug("Moviendose de reversa")
            throttle_value = 90 - self.rescale_input(value_no_drift, input_min_value=0, rescale_min_value=0, rescale_max_value=4)
        else:
            throttle_value = 90 + self.rescale_input(value_no_drift, input_min_value=0, rescale_min_value=0, rescale_max_value=4)
        self.get_logger().debug("right trigger changed: {} | {}".format(value_no_drift, throttle_value))
        self.throttle_value = throttle_value

    def on_left_stick_x_changed(self, left_stick_x):
        actual_value = left_stick_x
        value_no_drift = left_stick_x - \
            self.left_stick_drift if left_stick_x >= 0 else left_stick_x + self.left_stick_drift
        steering_value = self.rescale_input(value_no_drift) + 90
        # self.get_logger().info("on_left_stick_x_changed: {} | Drift: {}".format(actual_value, value_no_drift))
        if value_no_drift >= -0.2 and value_no_drift <= 0.2:
            # Ver si es necesario hacer 0 el angulo de movimiento (fijar a 90)
            steering_value = 90
            self.get_logger().debug("Vehiculo centrado | {} | Joy Data: {}".format(
                value_no_drift, steering_value))
        elif value_no_drift < -0.2:
            self.get_logger().debug("Vehiculo girando a la izquierda | {} | Joy Data: {}".format(
                value_no_drift, steering_value))
            # En este caso, se tiene que reescalar el valor de 0 a 45 para poder restarlo o sumarlo a los valores de la direccion
            # Recuerda 45 es totalmente a la izquierda, 90 es el medio, 135 es la derecha
            # Estos valores son negativos, van de 0 a -1, donde -1 es totalmente izquierda
            # Adicionalmente, settear una funcion suave o limite para no exceder el -1
        elif value_no_drift > 0.2:
            # joy_value = 90 + self.map_function_joy(value_no_drift)
            self.get_logger().debug("Vehiculo girando a la derecha | {} | Joy Data: {}".format(
                value_no_drift, steering_value))
            # En este caso, se tiene que reescalar el valor de 0 a 45 para poder restarlo o sumarlo a los valores de la direccion
            # Recuerda 45 es totalmente a la izquierda, 90 es el medio, 135 es la derecha
            # Estos valores son positivos, van de 0 a 1, donde 1 es totalmente derecha
            # Adicionalmente, settear una funcion suave o limite para no exceder el 1
        else:
            steering_value = 90
            self.get_logger().debug("Vehiculo en estado desconocido | {}".format(value_no_drift))
        # Enviar datos
        self.steering_value = steering_value

    def on_error(self, error):
        self.get_logger().error(f'Opps! an error occured: {error}')

    def __test_bridge__(self):
        test_angles = [80, 135, 105, 80, 75, 45, 80]
        # Probamos direccion
        for angle in test_angles:
            self.__send_controls__(angle, 90)
            sleep(0.5)
        # Probamos velocidad
        #for angle in test_angles:
        #    self.__send_controls__(90, angle)
        #    sleep(0.5)

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

    def disconnect_from_bridge(self):
        try:
            self.rc_bridge.close()
        except Exception as ex:
            self.get_logger().warning(
                "Ha ocurrido un error al comunicarse con el puente | {}".format(ex))


def main(args=None):
    rclpy.init(args=args)
    rc_car_subscriber = RCCarVanilla()

    # Use a try/finally block for clean shutdown
    try:
        rclpy.spin(rc_car_subscriber)
        sleep(0.01)
    except KeyboardInterrupt:
        if rc_car_subscriber.device:
            rc_car_subscriber.device.deactivate()
        if rc_car_subscriber.rc_bridge:
            rc_car_subscriber.disconnect_from_bridge()
    except Exception as ex:
        pass

    # Destroy the node and shutdown ROS 2
    rclpy.shutdown()


if __name__ == '__main__':
    main()
