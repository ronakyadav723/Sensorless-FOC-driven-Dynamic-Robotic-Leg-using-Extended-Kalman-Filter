from setuptools import setup

package_name = 'leg_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team',
    maintainer_email='team@example.com',
    description='Sensorless FOC robotic leg ROS 2 controller',
    license='MIT',
    tests_require=['pytest'],
      entry_points={
        'console_scripts': [
            'leg_controller = leg_controller.publisher_node:main',
            'bridge_node = leg_controller.bridge_node:main',
            'trajectory_reference_node_v2 = leg_controller.trajectory_reference_node_v3:main',
            'simulink_listener_node = leg_controller.simulink_listener_node:main',
            'torque_estimator_node = leg_controller.torque_estimator_node:main',
            'torque_bridge_node = leg_controller.torque_bridge_node:main',
               ],
           },
         )
