from setuptools import find_packages, setup
import subprocess

subprocess.run("sudo ip link set can0 down".split())
subprocess.run("sudo ip link set can0 up type can bitrate 250000 dbitrate 500000 berr-reporting on fd on".split())
package_name = 'odrive_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/odrv.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alsaibie',
    maintainer_email='ahmeduk0022@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odrive_control = odrive_test.odrive_control:main',

        ],
    },
)
