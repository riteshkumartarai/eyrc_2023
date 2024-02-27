from setuptools import find_packages, setup

package_name = 'bot_control_6'

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
    maintainer='subun',
    maintainer_email='soumitranayak71@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flower = bot_control_6.botControl:main',
            'bonus = bot_control_6.botControl_6b:main',
            'stop_service = bot_control_6.stopFlagService:main'
        ],
    },
)
