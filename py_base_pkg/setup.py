from setuptools import find_packages, setup

package_name = 'py_base_pkg'

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
    maintainer='ruben',
    maintainer_email='ruben.malacarne@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_node = py_base_pkg.py_node:main",
            "num_publisher_node = py_base_pkg.num_publisher_node:main",
            "num_subscriber_node = py_base_pkg.num_subscriber_node:main",
            "hardware_status_publisher_node = py_base_pkg.hw_status_publisher_node:main",
            "led_panel_node = py_base_pkg.led_panel_node:main",
            "battery_node = py_base_pkg.battery:main"
        ],
    },
)