from setuptools import setup
import os

package_name = 'motor_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', 'ament_python')]),
        ('share/' + package_name, ['package.xml']),
        # Add this line for your launch file
        (os.path.join('share', package_name, 'launch'), ['launch/motor_driver.xml']),
        (os.path.join('share', package_name, 'config'), ['config/motors.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonathan Boylan',
    maintainer_email='jboylan@fsu.edu',
    description='STARQ Motor Driver Package',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver_node = motor_driver.motor_driver_node:main'
        ],
    },
)
