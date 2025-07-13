from setuptools import setup
import os
from glob import glob

package_name = "jackal_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],                  
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ektoras",
    maintainer_email="ektoras.sofianopoulos@gmail.com",
    description="Jackal simulation bringup",
    license="Apache License 2.0",

    
    entry_points={
        "console_scripts": [
            
            "initial_pose_pub = jackal_bringup.initial_pose_pub:main",

            
            "pose_init_once = jackal_bringup.pose_init_once:main",
        ],
    },

    
    data_files=[
        
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),

        
        ("share/" + package_name, ["package.xml"]),

        
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
)
