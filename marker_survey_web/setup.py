from setuptools import find_packages, setup

package_name = 'marker_survey_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['static/*']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='marker_survey maintainers',
    maintainer_email='noreply@example.com',
    description='The browser interface: print marker sheets, identify markers from a photo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_survey_web = marker_survey_web.server:main',
        ],
    },
)
