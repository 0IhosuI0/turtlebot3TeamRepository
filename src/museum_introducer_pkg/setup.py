from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'museum_introducer_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/museum_guide_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghtn10000',
    maintainer_email='ghtn10000@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_recorder = museum_introducer_pkg.path_recorder:main',
            'recommand_path_drive = museum_introducer_pkg.recommand_path_drive:main',
            'qr_code_follower = museum_introducer_pkg.qr_code_follower:main',
            'test_qr = museum_introducer_pkg.test_qr:main',
        ],
    },
)
