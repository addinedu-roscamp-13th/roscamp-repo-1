from setuptools import find_packages, setup

package_name = 'ddagi_harvest'

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
    maintainer='kdh',
    maintainer_email='kdhkjh@gmail.com',
    description='Ddagi 수확 구현 (김동현) — AI 좌표 → send_coords 파지. 시나리오2 채택본.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'harvest_node = ddagi_harvest.harvest_node:main',
        ],
    },
)
