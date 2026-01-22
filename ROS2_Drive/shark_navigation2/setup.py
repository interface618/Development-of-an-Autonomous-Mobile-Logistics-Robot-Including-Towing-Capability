from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'shark_navigation2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),   # 파이썬 노드가 없다면 빈 패키지 디렉터리라도 하나 두는 것을 권장
    data_files=[
        # 패키지 인덱스 등록
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
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Navigation bringup (Nav2 + AMCL + Map) for SHARK robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        # 파이썬 실행 노드가 있으면 여기에 등록 (없으면 빈 리스트 유지)
        'console_scripts': [
            # 예) 'my_helper = shark_navigation2.my_helper:main',
        ],
    },
)
