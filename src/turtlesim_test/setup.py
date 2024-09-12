from setuptools import find_packages, setup

package_name = 'turtlesim_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/turtle_rqt_launch.py']),
        ('share/' + package_name, ['config/params.yaml']),
    ],
    scripts=["scripts/setspeed.sh","scripts/parse_velocity.sh","scripts/kill_turtle.sh"],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
