Necesitas descargar el librealsense 2.50.0

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_WITH_CUDA=true \
    -DBUILD_GRAPHICAL_EXAMPLES=true \
    -DBUILD_PYTHON_BINDINGS=true

make -j$(nproc)
sudo make install

Y El librealsense-ros-4.0.3 --> Este repo trae el 4.51.1 --> Funciona
Aplicale los parches en el Cmake de camera
Cambia la version y agrega el humble

Trae truco
rosdep install -i --from-path src --skip-keys="librealsense2" --rosdistro humble -y

Ejecutas un colcon build

Si marca que hay una incosistencia con el realsense
Asegurate de haber desinstalado las versiones apt, tener compilada la version 2.50. Tambien elimina las apt de ros-humble-librealsense* y ros-humble-realsense2*

Ya con eso limpia el area y recompila


### Lanzar el realsense como nodo independiente
ros2 launch realsense2_camera rs_launch.py \
    enable_infra1:=false \
    enable_infra2:=false \
    enable_infra:=false \
    enable_depth:=false \
    enable_gyro:=false \
    enable_accel:=false \
	rgb_camera.profile:=320x180x30 \
	rgb_camera.format:=RGB8

### Lanzar el realsense -- Util para el launch file propio
ros2 run realsense2_camera realsense2_camera_node --ros-args -p rgb_camera.profile:=320x180x30 -p rgb_camera.format:=RGB8 -p reconnect_timeout:=5.0

### V1 del subscriptor -- Socket
ros2 launch jetsoncar_v2 stream_image_subscriber.py --ros-args -r streaming_host:=192.168.1.14 -r streaming_port:=8089

ros2 run jetsoncar_v2 server_image --ros-args -p streaming_host:=192.168.1.14 -p streaming_port:=8089

### V2 del subscriptor -- RTMP * FFMPEG

ros2 run jetsoncar_v2 server_image --ros-args \
    -p stream_host:="192.168.1.52" \
    -p stream_path:="live/stream" \
    -p stream_fps:=30 \
    -p stream_res:="320x180" \
    -p stream_output_scale:="320x180" \
    -p use_yolo:=0 \
    -p yolo_model:="models/yolo11n_320_half_nms_cuda_21.onnx"

stream_res es la resolucion en el servicio del realsense

### Lanzar carrito
ros2 run jetsoncar_v2 rc_car_vanilla --ros-args -p left_stick_drift:=0.1
ros2 run jetsoncar_v2 rc_car_manual --ros-args -p left_stick_drift:=0.1


# Lanzar retransmision
docker run --rm -it --network=host bluenviron/mediamtx:1


Para el lidar, ocupa el Sweep-sdk de SweepSnow


ros2 run jetsoncar_v2 server_camera --ros-args \
    -p stream_host:="192.168.1.52" \
    -p stream_path:="live/stream" \
    -p stream_fps:=20 \
    -p stream_res:="1080x720" \
    -p stream_output_scale:="1080x720" \
    -p use_yolo:=0 \
    -p yolo_model:="models/yolo11n_320_half_nms_cuda_21.onnx"


ros2 run jetsoncar_v2 rc_car_vanilla --ros-args -p left_stick_drift:=0.1 -p load_camera:=1


if value_no_drift > 0.1:
            self.device.right_rumble.set(self.rescale_input(value_no_drift, 0, 1, 100, 180))
            self.device.left_rumble.set(self.rescale_input(value_no_drift, 0, 1, 100, 180))
        else:
            self.device.right_rumble.set(0)
            self.device.left_rumble.set(0)
        
        if value_no_drift > 0.1 and value_no_drift <= 0.45:
            self.device.right_trigger.effect.soft_rigidity()
        elif value_no_drift > 0.45 and value_no_drift <= 0.75:
            self.device.right_trigger.effect.medium_rigidity()
        elif value_no_drift > 0.75 and value_no_drift <= 1.0:
            self.device.right_trigger.effect.max_rigidity()
        else:
            self.device.right_trigger.effect.no_resistance()