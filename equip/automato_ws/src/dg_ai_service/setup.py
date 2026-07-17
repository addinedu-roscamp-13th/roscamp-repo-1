from setuptools import find_packages, setup

package_name = 'dg_ai_service'

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
    description='DG (DdaGoDdagi) AI Service — 정밀 분석 TCP 서버 (RP-50)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'analysis_server = dg_ai_service.analysis_server:main',
            'camera_viewer = dg_ai_service.camera_viewer:main',
        ],
    },
)
