from setuptools import setup
import os
from glob import glob

package_name = 'navigation_osr'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheely',
    maintainer_email='wheely@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_osr_node = navigation_osr.navigation_osr_node:main',
            'tf_listener = navigation_osr.tf_listener:main',
            'ocr= navigation_osr.ocr:main',
            'color_segmentation = navigation_osr.color_segmentation:main',
            'follow_path = navigation_osr.follow_path:main',
            'vision_prueba_1 = navigation_osr.vision_prueba_1:main',
        ],
    },
)
