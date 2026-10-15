import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'marker_localizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marker_survey maintainers',
    maintainer_email='noreply@example.com',
    description='The localizer: robot pose from a marker map, 6-DoF or planar.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_localizer = marker_localizer.localizer_node:main',
            'marker_localizer_evaluate = marker_localizer.tools.evaluate:main',
            'marker_localizer_score = marker_localizer.tools.score:main',
        ],
    },
)
