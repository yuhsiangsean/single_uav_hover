from setuptools import find_packages, setup

package_name = 'single_uav_hover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='a0929370607@gmail.com',
    description=(
        'Single UAV hover: arm/takeoff to a fixed hover altitude '
        'and land on keyboard command, PX4 SITL first then real '
        'hardware.'
    ),
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hover_control = single_uav_hover.hover_control:main',
            'hover_commander = single_uav_hover.hover_commander:main',
        ],
    },
)
