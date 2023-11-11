from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hb_task_1b'

setup(
    
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro*'))),
    
    (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes','*png'))),
    (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes','*stl'))),
    (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes','*dae'))),
    # (os.path.join('share', package_name), glob('*urdf.*')),
    # (os.path.join('share', package_name), glob('*meshes/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='subun',
    maintainer_email='subun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_node1=hb_task_1b.service_node:main'
        ],
    },
)
