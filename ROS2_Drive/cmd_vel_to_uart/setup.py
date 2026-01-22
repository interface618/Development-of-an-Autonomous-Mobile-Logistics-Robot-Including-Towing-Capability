from setuptools import setup

package_name = 'cmd_vel_to_uart'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Send cmd_vel to STM32 over UART',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_uart = cmd_vel_to_uart.cmd_vel_to_uart:main'
        ],
    },
)

