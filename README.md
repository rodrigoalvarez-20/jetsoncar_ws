cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_WITH_CUDA=true \
    -DBUILD_GRAPHICAL_EXAMPLES=true \
    -DBUILD_PYTHON_BINDINGS=true

make -j$(nproc)
sudo make install

Trae truco
rosdep install -i --from-path src --skip-keys="librealsense2" --rosdistro humble -y

ros2 launch realsense2_camera rs_launch.py \
	rgb_camera.profile:=320x180x30 \
	rgb_camera.format:=RGB8


ros2 launch jetsoncar_v2 stream_image_subscriber.py --ros-args -r streaming_host:=192.168.1.14 -r streaming_port:=8089

ros2 run jetsoncar_v2 server_image --ros-args -p streaming_host:=10.100.97.136 -p streaming_port:=8089

ros2 run jetsoncar_v2 rc_car_vanilla