import os
from glob import glob
from setuptools import setup

package_name = 'reference_square'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mamba',
    maintainer_email='denis.tal@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['odom_pub = reference_square.odom_pub:main',
                            'image_pub = reference_square.image_pub:main',
                            'camerainfo_pub = reference_square.camera_config_pub:main',
                            'reference_square = reference_square.reference_square:main'
                            ],
    },
)
