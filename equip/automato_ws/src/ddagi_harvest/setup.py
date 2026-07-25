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
    description='Ddagi 수확 구현 (김동현) — AI 좌표 → send_coords 파지. 경쟁 구현본.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 노드 추가 시 여기에 등록 (예: harvest_server = ddagi_harvest.harvest_server:main)
        ],
    },
)
