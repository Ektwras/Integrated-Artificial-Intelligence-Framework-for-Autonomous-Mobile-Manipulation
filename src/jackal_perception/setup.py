from setuptools import setup, find_packages

package_name = 'jackal_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
        ('share/' + package_name + '/params', ['jackal_perception/params/perception.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ektoras',
    maintainer_email='ektoras.sofianopoulos@gmail.com',
    description='YOLO-based perception server for Jackal',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detect_object_server = jackal_perception.detect_object_server:main',
        ],
    },
)
