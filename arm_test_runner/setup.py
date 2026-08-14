from setuptools import find_packages, setup

package_name = 'arm_test_runner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='albert',
    maintainer_email='13728790864@163.com',
    description=(
        'Automated joint trajectory tests for a ROS 2 robot arm controller'
    ),
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'trajectory_test_node = arm_test_runner.trajectory_test_node:main',
            'plot_results = arm_test_runner.plot_results:main',
        ],
    },
)
