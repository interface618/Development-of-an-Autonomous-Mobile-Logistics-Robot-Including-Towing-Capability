from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'shark_multi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),

        # package.xml 설치
        (os.path.join('share', package_name), ['package.xml']),

        # launch, config, map, rviz 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'map'),    glob('map/*')),
        (os.path.join('share', package_name, 'rviz'),   glob('rviz/*')),

        # 테스트 파일을 유지할 경우
        (os.path.join('share', package_name, 'test'),   glob('test/*')),
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
            'initialpose=shark_multi.initialpose:main',
            'laser_scan_cropper=shark_multi.laser_scan_cropper:main',
            'mode_process_manager=shark_multi.mode_process_manager:main',
            'cmd_vel_switching=shark_multi.cmd_vel_switching:main',
        ],
    },
)
