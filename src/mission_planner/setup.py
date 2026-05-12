import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mission_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web_static'),
            glob('mission_planner/web_static/*')),
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
            'mission_planner = mission_planner.mission_executor:main',
            'mission_gui     = mission_planner.mission_gui:main',
            'mission_web     = mission_planner.mission_web_server:main',
        ],
    },
)
