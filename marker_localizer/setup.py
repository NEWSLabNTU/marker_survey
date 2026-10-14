from setuptools import find_packages, setup

package_name = 'marker_localizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marker_survey maintainers',
    maintainer_email='noreply@example.com',
    description='The localizer: robot pose from a marker map, 6-DoF or planar. Placeholder (phase 3).',
    license='Apache-2.0',
    tests_require=['pytest'],
)
