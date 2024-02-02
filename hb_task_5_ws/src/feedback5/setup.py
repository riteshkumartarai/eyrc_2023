from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'feedback5'

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
            'feedback_node = feedback5.feedback_node:main',
            'image_proc = feedback5.image_proc:main',
            'viz_pose = feedback5.viz_pose:main',
        ],
    },
)
