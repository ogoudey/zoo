from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'gz_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/gz_launcher.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olin',
    maintainer_email='olin.goog@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gz_interface = gz_interface.gz_interface:main'
        ],
    },
)
