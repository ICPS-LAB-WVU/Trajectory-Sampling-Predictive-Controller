from setuptools import setup

package_name = 'tspc_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['../launch/tspc.launch.py']),
        ('share/' + package_name + '/config', ['../config/tspc_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Elgouhary',
    maintainer_email='mae00018@mix.wvu.edu',
    description='ROS2 TSPC controller with trajectory sampling, raceline tracking, and LiDAR-reactive obstacle avoidance.',
    license='TODO: Choose a license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tspc_node = tspc_controller.tspc_node:main',
        ],
    },
)
