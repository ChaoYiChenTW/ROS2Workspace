from setuptools import setup

package_name = "device_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HDA",
    maintainer_email="HDA@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "push_button = device_controller.push_button:main",
            "led_toggle = device_controller.led_toggle:main",
            "potentiometer = device_controller.potentiometer:main",
            "led_dimmer = device_controller.led_dimmer:main",
        ],
    },
)
