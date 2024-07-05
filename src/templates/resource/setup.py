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
        # Include all launch files
        {{#if include_launch}}
        (os.path.join('share', package_name,'launch'), glob('launch/*.py')),
        {{/if}}
        {{#if include_urdf}}
        (os.path.join('share', package_name,'urdf'), glob('urdf/*')),
        {{/if}}
        {{#if include_meshes}}
        (os.path.join('share', package_name,'meshes'), glob('meshes/*'))
        {{/if}}
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='{{package_maintainer_name}}',
    maintainer_email='{{package_maintainer_email}}',
    description='{{package_description}}',
    license='{{package_license}}',
    entry_points={
    },
)