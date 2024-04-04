import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'autonom_auv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'photos'), glob('photos/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')), 
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Description of your package',
    license='License of your choice',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movement_node = autonom_auv.movement_node:main',
            'm_pipeline_node = autonom_auv.m_pipeline_node:main',
            'irl_m_pipeline_node = autonom_auv.irl_m_pipeline_node:main',
            'm_bench_node = autonom_auv.m_bench_node:main',
            'up_down_node = autonom_auv.up_down_node:main',
            'dvl_movement_node = autonom_auv.dvl_movement_node:main',
            'USB_Camera_node = autonom_auv.USB_Camera_node:main',
        ],
    },
)
