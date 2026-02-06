from setuptools import setup

package_name = "hbr_joint_state_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HBR",
    maintainer_email="n/a",
    description="Course joint state publisher + GUI that reads /robot_description and publishes /joint_states.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "hbr_joint_state_publisher_gui = hbr_joint_state_publisher.hbr_joint_state_publisher_gui:main",
            # Optional: add a non-GUI publisher here if you want
            # "hbr_joint_state_publisher = hbr_joint_state_publisher.hbr_joint_state_publisher:main",
        ],
    },
)
