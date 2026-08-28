import glob

from setuptools import find_packages, setup

package_name = 'system_admin_app'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 배경 SLAM 맵 (설치 실행 시에도 찾도록 패키징)
        ('share/' + package_name + '/maps', glob.glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kdh',
    maintainer_email='kdhkjh@gmail.com',
    description='Automato System Admin APP (QT) — E0 상시 모니터링 루프 대시보드',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 메인 대시보드 (QT)
            'system_admin_app = system_admin_app.main:main',
            # 개발/시연용 모의 텔레메트리 발행기
            'sim_publisher = system_admin_app.ros.sim_publisher:main',
            # 개발/시연용 모의 ACS (유지보수 명령 수신 서버)
            'mock_acs = system_admin_app.ros.mock_acs:main',
        ],
    },
)
