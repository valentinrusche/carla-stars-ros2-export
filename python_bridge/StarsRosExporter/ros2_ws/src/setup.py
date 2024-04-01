from setuptools import setup

package_name = 'stars_ros_exporter'

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        package_name,
        package_name + "/stars",
        package_name + "/util",
        package_name + "/xodr",
        package_name + "/nodes",
        package_name + "/nodes/stars_implementations"
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Valentin Rusche',
    maintainer_email='valentin.rusche@udo.edu',
    description='Bridge to read STARS ROS topics and export them to STARS compatible json format',
    license='AGPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stars_ros_exporter = stars_ros_exporter.main:main',
        ],
    },
)
