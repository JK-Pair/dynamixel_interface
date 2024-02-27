from setuptools import find_packages, setup

package_name = 'dynamixel_interface'

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
    maintainer='jk-pair',
    maintainer_email='thipawan2112@gmail.com',
    description='This package serves as a bridge to connect control algorithms directly to Dynamixel motors',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dxl_pub = dynamixel_interface.dynamixel_publisher:main',
            'dxl_sub = dynamixel_interface.dynamixel_subscriber:main'
        ],
    },
)
