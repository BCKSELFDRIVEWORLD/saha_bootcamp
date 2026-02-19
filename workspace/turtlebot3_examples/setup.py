from setuptools import find_packages, setup

package_name = 'turtlebot3_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='todo@todo.com',
    description='TurtleBot3 example nodes covering topics, services, and actions',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_driver = turtlebot3_examples.square_driver:main',
            'odom_listener = turtlebot3_examples.odom_listener:main',
            'rotate_server = turtlebot3_examples.rotate_server:main',
            'rotate_client = turtlebot3_examples.rotate_client:main',
            'patrol_server = turtlebot3_examples.patrol_server:main',
            'patrol_client = turtlebot3_examples.patrol_client:main',
        ],
    },
)
