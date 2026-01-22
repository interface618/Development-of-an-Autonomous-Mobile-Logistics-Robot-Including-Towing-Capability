import os
from glob import glob
from setuptools import setup

package_name = 'shark_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- [추가된 부분 시작] ---
        # launch 디렉토리 안의 모든 .launch.py 파일을 설치합니다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # config 디렉토리 안의 모든 .yaml 파일을 설치합니다.
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # --- [추가된 부분 끝] ---
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Utility nodes for shark project',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosout_trigger = shark_utils.rosout_trigger:main',
        ],
    },
)
