from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'goat_behavior_tree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simon',
    maintainer_email='simonroy99@hotmail.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_server = scripts.navigate_server:main',
            'pick_server = scripts.pick_server:main',
            'place_server = scripts.place_server:main',
            'locate_server = scripts.locate_server:main',
            'assist_server = scripts.assist_server:main',
            'wait_server = scripts.wait_server:main',
            'execute_server = scripts.execute_server:main',
        ],
    },
) 