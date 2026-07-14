from glob import glob

from setuptools import find_packages, setup

package_name = "jetcar_edge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/edge.yaml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=[
        "setuptools",
        "websocket-client",
        "PyYAML",
        "requests",
        "fastapi",
        "uvicorn",
        "Pillow",
        "numpy",
    ],
    zip_safe=True,
    maintainer="JetCar Team",
    maintainer_email="dev@example.com",
    description="Jetson-side camera and sensor upload bridge for JetCar AI inference.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "edge_upload_node = jetcar_edge.edge_upload_node:main",
            "slam_web_api = jetcar_edge.slam_web_api:main",
        ],
    },
)
