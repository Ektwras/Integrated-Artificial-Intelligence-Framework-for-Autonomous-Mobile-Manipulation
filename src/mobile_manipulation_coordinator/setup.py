from setuptools import find_packages, setup

package_name = 'mobile_manipulation_coordinator'

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
    maintainer='ektoras',
    maintainer_email='ektoras.sofianopoulos@gmail.com',
    description='Coordinator node to sequence Nav2 and MoveIt2 actions',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             # This makes ros2 run mobile_manipulation_coordinator coordinator work:
            'coordinator = mobile_manipulation_coordinator.coordinator:main',
        ],
    },
)