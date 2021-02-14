from setuptools import setup

package_name = 'intro_py_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Carlos Manzanares Serrano',
    maintainer_email='jc.manzanares.serrano@gmail.com',
    description='Package with examples for ROS2 (rclpy)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_node = intro_py_ros2.simple_node:main'
        ],
    },
)
