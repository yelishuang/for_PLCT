from setuptools import setup

package_name = 'ros_chat'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 node for WebSocket client and message publishing.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_chater = ros_chat.ros_chater:main',
        ],
    },
)
