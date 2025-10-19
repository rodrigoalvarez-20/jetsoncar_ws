import rclpy
from rclpy.node import Node
from time import sleep
from dualsense_controller import DualSenseController

class RCCarVanilla(Node):
    
    def __init__(self):
        super().__init__('rc_car_vanilla')
        
        self.device = None
        self.get_logger().info("Inicializando conexión con dispositivo...")
        self.connect_to_device()
        self.device.activate()
        #self.device.lightbar.set_color_red()

        
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
            controller.btn_cross.on_down(self.on_cross_btn_pressed)
            controller.btn_cross.on_up(self.on_cross_btn_released)
            controller.left_trigger.on_change(self.on_left_trigger)
            controller.left_stick_x.on_change(self.on_left_stick_x_changed)
            # register the error callback
            controller.on_error(self.on_error)
            controller.lightbar.set_color_red()
            self.device = controller

            
    
    def stop(self):
        pass


    # callback, when cross button is pressed, which enables rumble
    def on_cross_btn_pressed(self):
        self.get_logger().info("Boton X presionado")
        #self.device.left_rumble.set(255)
        #self.device.right_rumble.set(255)


    # callback, when cross button is released, which disables rumble
    def on_cross_btn_released(self):
        self.get_logger().info("Boton X liberado")
        #self.device.left_rumble.set(0)
        #self.device.right_rumble.set(0)

    def on_left_trigger(self, value):
        self.get_logger().info("left trigger changed: {}".format(value))
        #self.device.left_rumble.set(127)
        #self.device.left_trigger.effect.max_rigidity()
        #self.device.left_trigger.simple_vibration(start_position=0, amplitude=255, frequency=8)
        #self.device.left_trigger.effect


    def on_left_stick_x_changed(self, left_stick_x):
        self.get_logger().info("on_left_stick_x_changed: {}".format(left_stick_x))

    # callback, when unintended error occurs,
    # i.e. physically disconnecting the controller during operation
    # stop program
    def on_error(self, error):
        self.get_logger().error(f'Opps! an error occured: {error}')
        

def main(args=None):
    rclpy.init(args=args)
    rc_car_subscriber = RCCarVanilla()
    
    # Use a try/finally block for clean shutdown
    try:
        rclpy.spin(rc_car_subscriber)
        sleep(0.001)
    except KeyboardInterrupt:
        if rc_car_subscriber.device:
            rc_car_subscriber.device.deactivate()
    except Exception as ex:
        pass
    
    # Destroy the node and shutdown ROS 2
    rclpy.shutdown()
        

if __name__ == '__main__':
    main()