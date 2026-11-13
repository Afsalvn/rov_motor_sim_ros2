from setuptools import find_packages, setup

package_name = 'rov_motor_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch + config don't get installed unless they are listed here
        ('share/' + package_name + '/launch',
            ['launch/motor_sim.launch.py']),
        ('share/' + package_name + '/config',
            ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Afsalvn',
    maintainer_email='btechproject531@gmail.com',
    description=(
        'Converts cmd_vel velocity commands to thruster RPM targets, '
        'simulates the motor response and publishes RPM telemetry.'
    ),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bridge_node = rov_motor_sim.bridge_node:main',
            'motor_sim_node = rov_motor_sim.motor_sim_node:main',
        ],
    },
)
