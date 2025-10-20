from setuptools import find_packages, setup

package_name = 'mobile_manipulation_coordinator'

setup(
    name=package_name,
    version='0.1.0',
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
    description='Coordinator node(s) to sequence Nav2 navigation and DetectObject perception actions.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Classic coordinator (current behavior: NAV -> Detect)
            'coordinator = mobile_manipulation_coordinator.coordinator:main',

            # New variant you’ll paste as coordinator_v1.py (queue/dual-goal capable)
            'coordinator_v1 = mobile_manipulation_coordinator.coordinator_v1:main',
        ],
    },
)
