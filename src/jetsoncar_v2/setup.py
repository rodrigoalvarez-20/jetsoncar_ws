from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'jetsoncar_v2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rodrigo',
    maintainer_email='rodrigoalvarez449@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "local_image = jetsoncar_v2.local_image_subscriber:main",
            "server_image = jetsoncar_v2.stream_image_subscriber:main",
            "rc_car_vanilla = jetsoncar_v2.rc_car_vanilla:main",
            "rc_car_manual = jetsoncar_v2.rc_car_manual:main",
            "server_camera = jetsoncar_v2.stream_camera_subscriber:main"
        ],
    },
)
