from glob import glob

from setuptools import find_packages, setup

package_name = 'ddago_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boyeon',
    maintainer_email='lboyeon1223@gmail.com',
    description='DdaGo Control Service — 주행 로봇 제어/텔레메트리 노드',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_publisher = ddago_control.telemetry_publisher:main',
            'navigate_server = ddago_control.navigate_server:main',
            'camera_node = ddago_control.camera_node:main',
        ],
    },
)
