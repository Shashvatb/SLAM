from setuptools import find_packages, setup

package_name = 'pub_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch file add
        ('share/' + package_name + '/launch', ['launch/simple.launch.py']),
        # end of launch file add
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
            # added section
            'minimal_publisher = pub_sub.publisher:main',
            'minimal_subscriber = pub_sub.subscriber:main',
            # end of added section
        ],
    },
)
