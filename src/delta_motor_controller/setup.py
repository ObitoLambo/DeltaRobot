from setuptools import find_packages, setup

package_name = 'delta_motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='s4mb4th',
    maintainer_email='s4mb4th@todo.todo',
    description='Delta motor controller',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'valve_can_node=delta_motor_controller.valve_can_node:main',
        ],
    },
)
