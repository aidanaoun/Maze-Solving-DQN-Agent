from setuptools import find_packages, setup

package_name = 'mazeRobot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        # leave TensorFlow out unless you truly want pip to fetch it here
        # 'tensorflow',
    ],
    zip_safe=True,
    maintainer='aidan',
    maintainer_email='aidan@todo.todo',
    description='Controller, sensors, and brain nodes for the maze robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'brain_node = mazeRobot.brain_node:main',
            'sensors_node = mazeRobot.sensors_node:main',
            'motor_controller_node = mazeRobot.motor_controller_node:main',
            'pose_reader_node = mazeRobot.pose_reader_node:main'
        ],
    },
)

