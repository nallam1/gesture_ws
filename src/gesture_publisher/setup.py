from setuptools import find_packages, setup

package_name = 'gesture_publisher'

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
    description='ROS 2 node that publishes recognized hand gestures for robot teleoperation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_publisher = gesture_publisher.gesture_publisher_node:main',
	    'manual_gesture_publisher = gesture_publisher.manual_gesture_publisher:main',
        ],
    },
)
