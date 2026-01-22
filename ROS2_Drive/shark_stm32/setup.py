from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'shark_stm32'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lee',
    maintainer_email='lee@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_listener=shark_stm32.cmd_vel:main',
            'joint_states_listener=shark_stm32.joint_states:main',
            'stm32_serial=shark_stm32.send_stm32:main',
            'teleop_keyboard=shark_stm32.teleop_keyboard:main'
        ],
    },
)