from setuptools import find_packages, setup

package_name = 'so101_bridge'

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
    maintainer='arka',
    maintainer_email='arkababu2003@gmail.com',
    description='SO101 ROS2 bridge for RViz2 visualization',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'so101_joint_state_publisher = so101_bridge.so101_joint_state_publisher:main',
        ],
    },
)
    