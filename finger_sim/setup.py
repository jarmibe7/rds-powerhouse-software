from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'finger_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py') + glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    test_suite='tests',
    zip_safe=True,
    maintainer='Jared Berry',
    maintainer_email='jarmibe7@gmail.com',
    description='Drake-based simulation of the Powerhouse finger',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'main = finger_sim.main:main',
        ],
    },
)