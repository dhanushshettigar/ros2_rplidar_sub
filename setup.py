from setuptools import find_packages, setup

package_name = 'ros2_rplidar_sub'

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
    maintainer='Dhanush Shettigar',
    maintainer_email='dhanushshettigar90@gmail.com',
    description='Package to print rplidar scanner values',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_listener=ros2_rplidar_sub.read:main',
            'detect_obj=ros2_rplidar_sub.obstacle:main',
 ],
    },
)
