from setuptools import find_packages, setup

package_name = 'service_robot'

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
    maintainer='test',
    maintainer_email='test@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer        = service_robot.explorer:main',
            'map_auto_saver  = service_robot.map_auto_saver:main',
            'teleop_mux      = service_robot.teleop_mux:main',
        ],
    },
)
