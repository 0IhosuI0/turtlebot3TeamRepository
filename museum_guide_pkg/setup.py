from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'museum_guide_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [os.path.join('launch', 'museum_full_system_launch.py')]),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'data'), glob('data/*.json')),
        (os.path.join('share', package_name, 'logs'), glob('logs/*.log')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.world')),
        (os.path.join('share', package_name, 'profiles'), glob('profiles/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghtn10000',
    maintainer_email='ghtn10000@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'museum_guide_node = museum_guide_pkg.main:main',
        ],
    },
)