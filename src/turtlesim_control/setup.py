from setuptools import find_packages, setup

package_name = 'turtlesim_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/turtle_control_launch.py']),
        ('share/' + package_name, ['launch/turtle_control_line_launch.py']),
        ('share/' + package_name, ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtlesim_control_node = turtlesim_control.control:main",
            "turtlesim_sender_node = turtlesim_control.goal_sender:main",
            "turtlesim_line_node = turtlesim_control.control_line:main",
        ],
    },
)
