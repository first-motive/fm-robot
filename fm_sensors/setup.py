from setuptools import find_packages, setup

package_name = "fm_sensors"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    # opencv + numpy are pip/rosdep provided; the camera reader needs them at runtime.
    install_requires=["setuptools", "numpy", "opencv-python"],
    zip_safe=True,
    maintainer="First Motive",
    maintainer_email="nish@ubundi.co.za",
    description="First Motive multi-sensor capture layer: camera publisher (image_raw + camera_info)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sensor_node = fm_sensors.sensor_node:main",
            # Camera publisher: phone-over-IP / device -> image_raw + camera_info. The head
            # camera today; run one instance per wrist camera later.
            "camera_node = fm_sensors.camera_node:main",
        ],
    },
)
