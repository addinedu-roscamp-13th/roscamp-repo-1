import glob

from setuptools import find_packages, setup

package_name = 'mycobot_280_pick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='u2',
    maintainer_email='u2@todo.todo',
    description='myCobot 280 좌표 입력 -> TF2 변환 -> MoveIt2 목표 플래닝 노드 패키지',
    license='BSD',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coord_to_goal_node = mycobot_280_pick.coord_to_goal_node:main',
        ],
    },
)
