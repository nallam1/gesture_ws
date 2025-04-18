from setuptools import find_packages, setup

package_name = 'gesture_subscriber'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nader',
    maintainer_email='nader.allam@mail.utoronto.ca',
    description='ROS 2 node that subscribes to hand gesture commands for robot teleoperation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_subscriber = gesture_subscriber.gesture_subscriber_node:main',
        ],
    },
)
