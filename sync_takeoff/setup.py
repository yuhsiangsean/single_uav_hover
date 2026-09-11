from setuptools import setup

package_name = 'sync_takeoff'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='a0929370607@gmail.com',
    description='讓多台 PX4 SITL 無人機同時 arm + offboard + 起飛的 ROS 2 節點',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sync_takeoff = sync_takeoff.sync_takeoff:main',
            'leader_follower = sync_takeoff.leader_follower:main',
        ],
    },
)
