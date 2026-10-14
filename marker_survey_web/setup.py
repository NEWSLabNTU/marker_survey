from setuptools import find_packages, setup

package_name = 'marker_survey_web'

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
    description='The browser interface for a site survey. Placeholder (phase 5).',
    license='Apache-2.0',
    tests_require=['pytest'],
)
