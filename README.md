cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_WITH_CUDA=true \
    -DBUILD_GRAPHICAL_EXAMPLES=true \
    -DBUILD_PYTHON_BINDINGS=true

make -j$(nproc)
sudo make install


ros2 launch realsense2_camera rs_launch.py \
	rgb_camera.profile:=640x480x30 \
	rgb_camera.format:=RGB8