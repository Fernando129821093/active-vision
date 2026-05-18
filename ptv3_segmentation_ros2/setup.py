from setuptools import setup

package_name = 'ptv3_segmentation_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NBV-PPO',
    maintainer_email='fayalasarria@gmail.com',
    description='PTv3 semantic segmentation ROS2 node (geometry-only, LiDAR input)',
    license='BSD',
    entry_points={
        'console_scripts': [
            'ptv3_node = ptv3_segmentation_ros2.ptv3_node:main',
        ],
    },
)
