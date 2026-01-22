from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'shark_urdf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 인덱스 등록
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),

        # package.xml 설치
        (os.path.join('share', package_name), ['package.xml']),

        # launch/, urdf/, meshes/ 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), #꼭 필요한놈들
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')), #꼭 필요한놈들
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')), #꼭 필요한놈들
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lee',
    maintainer_email='lee@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
