from glob import glob

from setuptools import find_packages, setup

package_name = 'automato_control_service'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='geonsulee',
    maintainer_email='zkffkejr145@gmail.com',
    description='Automato Control Service — Web/앱 → DG(OpHarvest) 중계',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # RP-78: 순찰 제어 노드 + 순찰 HTTP API(FastAPI)를 한 프로세스로 기동
            'patrol_node = automato_control_service.patrol_node:main',
            # RP-90: 텔레메트리 WebSocket 서버 — fleet 구독 → 1Hz 방송(독립 프로세스)
            'telemetry_ws_node = '
            'automato_control_service.telemetry_ws_node:main',
            # RP-114: 로봇별 텔레메트리 취합 → QT 대시보드 발행(독립 프로세스)
            'fleet_telemetry_aggregator = '
            'automato_control_service.fleet_telemetry_aggregator:main',
            # RP-78 실물 테스트 전용 임시 스탠드인 (실제 DG Control Service 아님)
            'dg_stub = '
            'automato_control_service.test_harness.dg_stub:main',
            'patrol_bridge = '
            'automato_control_service.test_harness.patrol_bridge:main',
            # A 티어 전용: 물리 로봇 없이 가짜 로봇 텔레메트리 발행 (로봇 1대로 다중 로봇 검증)
            'fake_telemetry = '
            'automato_control_service.test_harness.fake_telemetry:main',
        ],
    },
)
