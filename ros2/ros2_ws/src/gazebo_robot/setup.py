from setuptools import find_packages, setup

package_name = 'gazebo_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gazebo_robot_xacro.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/gazebo_robot_xacro.urdf.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='shashvat.bharti@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'forward_drive_publisher = gazebo_robot.forward_drive_publisher:main'
        ],
    },
)
