from setuptools import setup

package_name = 'uancabot_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='uancabot',
    maintainer_email='uancabot@example.com',
    description='UancaBot serial odometry node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uancabot_node = uancabot_odometry.uancabot_serial_node:main',
            'path_executor = uancabot_odometry.path_executor_node:main',
        ],
    },
)
