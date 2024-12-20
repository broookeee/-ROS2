from setuptools import setup

package_name = "mngr_fr_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "img_sub = " + package_name + ".img_sub:main",
            "cam_pub = " + package_name + ".cam_pub:main",
            "update_users = " + package_name + ".update_users:main",
            "greet_user = " + package_name + ".greet_user:main",
        ],
    },
    description="A face recognition package for the office manager robot",
)