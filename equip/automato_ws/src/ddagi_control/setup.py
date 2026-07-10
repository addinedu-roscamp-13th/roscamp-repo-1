from setuptools import find_packages, setup

package_name = 'ddagi_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hskim',
    maintainer_email='finekim67@gmail.com',
    description='Ddagi Control Service — 로봇팔 제어 노드',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_publisher = ddagi_control.telemetry_publisher:main',
            'harvest_server = ddagi_control.harvest_server:main',
        ],
    },
)
