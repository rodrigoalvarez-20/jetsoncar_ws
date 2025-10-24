Necesitas descargar el librealsense 2.50.0

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_WITH_CUDA=true \
    -DBUILD_GRAPHICAL_EXAMPLES=true \
    -DBUILD_PYTHON_BINDINGS=true

make -j$(nproc)
sudo make install

Y El librealsense-ros-4.0.3
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
	rgb_camera.profile:=320x180x30 \
	rgb_camera.format:=RGB8

### Lanzar el realsense -- Util para el launch file propio
ros2 run realsense2_camera realsense2_camera_node --ros-args -p rgb_camera.profile:=320x180x30 -p rgb_camera.format:=RGB8 -p reconnect_timeout:=5.0

### V1 del subscriptor -- Socket
ros2 launch jetsoncar_v2 stream_image_subscriber.py --ros-args -r streaming_host:=192.168.1.14 -r streaming_port:=8089

ros2 run jetsoncar_v2 server_image --ros-args -p streaming_host:=192.168.1.14 -p streaming_port:=8089

### V2 del subscriptor -- RTMP * FFMPEG

ros2 run jetsoncar_v2 server_image --ros-args \
    -p stream_host:="192.168.1.15" \
    -p stream_port:=8554 \
    -p stream_path:="stream/detections" \
    -p stream_fps:=30 \
    -p stream_res:="640x480"


### Lanzar carrito
ros2 run jetsoncar_v2 rc_car_vanilla



docker run --rm -it --network=host bluenviron/mediamtx:1
