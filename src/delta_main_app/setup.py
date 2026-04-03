from setuptools import find_packages, setup

package_name = 'delta_main_app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/delta_main.launch.py',
            'launch/blind_pick_place.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='s4mb4th',
    maintainer_email='s4mb4th@todo.todo',
    description='Delta robot main integration app',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'main_app        = delta_main_app.main_app:main',
            'blind_pick_place = delta_main_app.blind_pick_place:main',
        ],
    },
)