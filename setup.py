import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'autonom_auv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    #package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'photos'), glob('photos/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),    
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
            'pipeline_image_node = autonom_auv.pipeline_image_node:main',
            'front_cam_node = autonom_auv.front_cam_node:main',
            'fake_controller_node = autonom_auv.fake_controller_node:main',
            'up_down_node = autonom_auv.up_down_node:main',
            'blind_movement_node = autonom_auv.blind_movement_node:main',
        ],
    },
)
