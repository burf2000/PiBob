from setuptools import setup
from glob import glob

package_name = 'pibob_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='Simon Burfield',
    maintainer_email='simon@burf.co',
    description='Bridge ROS 2 joint commands to the PiBob Flask servo API.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'servo_bridge = pibob_driver.servo_bridge_node:main',
        ],
    },
)
