from setuptools import find_packages, setup

package_name = 'px4_ros_com'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yuhsiangsean.en15@nycu.edu.tw',
    description='ROS 2 communication with PX4',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_listener = px4_ros_com.odometry_listener:main',
	    'offboard_test = px4_ros_com.offboard_test:main',
	    'offboard_forward_2m = px4_ros_com.offboard_forward_2m:main',
        ],
    },
)
