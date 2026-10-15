import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'marker_survey'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'schema'), glob('schema/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marker_survey maintainers',
    maintainer_email='noreply@example.com',
    description='Map fiducial markers in place with a moving camera; the marker map file.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_survey = marker_survey.survey_node:main',
            'marker_survey_solve = marker_survey.tools.solve:main',
            'marker_survey_synthetic = marker_survey.tools.synthetic:main',
            'marker_survey_compare = marker_survey.tools.compare:main',
            'marker_map_check = marker_survey.tools.map_check:main',
            'marker_map_import = marker_survey.tools.map_import:main',
            'marker_plan = marker_survey.tools.plan:main',
            'marker_sheets = marker_survey.tools.sheets:main',
            'marker_identify = marker_survey.tools.identify:main',
        ],
    },
)
