from setuptools import find_packages, setup

package_name = 'nav2goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='k2c1',
    maintainer_email='119794073+KIM2C1@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2goal = nav2goal.nav2goal:main',
            'robot_follower_nav2=nav2goal.robot_follower_nav2:main'
        ],
    },
)
