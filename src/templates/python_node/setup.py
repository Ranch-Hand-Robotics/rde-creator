import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = '{{package_name}}'

setup(
    name=package_name,
    version='{{package_version}}',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), glob('package.xml')),
        {{#if include_launch}}
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        {{/if}}
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='{{package_maintainer_name}}',
    maintainer_email='{{package_maintainer_email}}',
    description='{{package_description}}',
    license='{{package_license}}',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '{{node_name}} = {{package_name}}.{{node_name}}:main',
        ],
    },
)