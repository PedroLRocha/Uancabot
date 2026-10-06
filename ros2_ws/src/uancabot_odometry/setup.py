import os
from glob import glob
from setuptools import setup

package_name = 'uancabot_odometry'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*.html')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='uancabot',
    maintainer_email='uancabot@example.com',
    description='UancaBot motor bridge, odometry, route generation and path following',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_bridge = uancabot_odometry.motor_bridge_node:main',
            'path_generator = uancabot_odometry.path_generator_node:main',
            'path_follower = uancabot_odometry.path_follower_node:main',
            'route_ui = uancabot_odometry.route_ui_node:main',
        ],
    },
)
