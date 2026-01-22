import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lider'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 디렉터리와 그 안의 모든 launch 파일을 포함시킵니다.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='A ROS2 package to publish lider data from the STM32 serial node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # '실행파일_이름 = 패키지이름.파이썬파일_이름:main' 형식입니다.
            'stm32_serial_node = lider.stm32_serial_node:main',
        ],
    },
)
