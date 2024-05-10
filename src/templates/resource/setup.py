import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'bb_description'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), glob('package.xml')),
        # Include all launch files
        (os.path.join('share', package_name,'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name,'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name,'meshes'), glob('meshes/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lou Amadio',
    maintainer_email='lou@polyhobbyist.com',
    description='ROS 2 BB-8',
    license='MIT',
    entry_points={
    },
)