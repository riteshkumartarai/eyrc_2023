from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bot_control5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='subun',
    maintainer_email='soumitranayak71@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = bot_control5.control_node:main',
            'control_node1 = bot_control5.control_node1:main',
            'control_node2 = bot_control5.control_node2:main',
            'control_node3 = bot_control5.control_node3:main',
            'mini_theme = bot_control5.mini_theme:main',
            'stopflag = bot_control5.stopService:main'
        ],
    },
)
