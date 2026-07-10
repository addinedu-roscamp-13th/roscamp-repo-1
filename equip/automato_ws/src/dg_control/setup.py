from setuptools import find_packages, setup

package_name = 'dg_control'

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
    maintainer='heeseog',
    maintainer_email='finekim67@gmail.com',
    description='DG Control Service (HQ) 순찰 오케스트레이터 본체',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hq_node = dg_control.hq_node:main',
        ],
    },
)
