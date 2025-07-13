from setuptools import setup

package_name = "llm_interface"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],                 
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
         ["launch/chat.launch.py"]),
        ("share/" + package_name + "/config",
         ["llm_interface/config/landmarks.yaml"]),
    ],
    install_requires=[
        "setuptools",
        "openai",
        "tf-transformations",
        "PyYAML",
    ],
    zip_safe=True,
    maintainer="ektoras",
    maintainer_email="ektoras.sofianopoulos@gmail.com",
    description=(
        "Chat‑LLM node that translates natural‑language tasks into "
        "geometry_msgs/PoseStamped goals."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "chat = llm_interface.chat:main",
        ],
    },
)
