import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dg_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='heeseog',
    maintainer_email='finekim67@gmail.com',
    description='DG Control Service(HQ) 테스트용 상대편 시뮬레이터 4종',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'acs_sim = dg_sim.acs_sim:main',
            'ddago_sim = dg_sim.ddago_sim:main',
            'ddagi_sim = dg_sim.ddagi_sim:main',
            'dg_ai_sim = dg_sim.dg_ai_sim:main',
        ],
    },
)
