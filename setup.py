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
        (os.path.join('share', package_name, 'pictures'), glob('pictures/*')),
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
            'movement = autonom_auv.movement:main',
            'applyForce = autonom_auv.applyForce:main',
            'imageHandler = autonom_auv.imageHandler:main',
            'controller = autonom_auv.controller:main',
            'fakeController = autonom_auv.fakeController:main',
            'relativeForce = autonom_auv.relativeForce:main',
        ],
    },
)
