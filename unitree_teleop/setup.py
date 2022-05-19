from setuptools import find_packages
from setuptools import setup
import os
from glob import glob

package_name = 'unitree_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=['unitree_teleop.unitree_teleop_twist', 
                'unitree_teleop.unitree_teleop_set_control_mode'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('./launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    maintainer='Sotaro Katayama',
    maintainer_email='sotaro.katayama@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Teleop for Unitree from the keyboard.'
    ),
    entry_points={
        'console_scripts': [
            'unitree_teleop_twist = unitree_teleop.unitree_teleop_twist:main',
            'unitree_teleop_set_control_mode = unitree_teleop.unitree_teleop_set_control_mode:main'
        ],
    },
)
