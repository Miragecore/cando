from setuptools import setup,find_packages
import os
import glob
from pathlib import Path

package_name = 'anynet_disparity'


def glob_fix(package_name, glob):
    # this assumes setup.py lives in the folder that contains the package
    package_path = Path(f'./{package_name}').resolve()
    return [str(path.relative_to(package_path)) 
            for path in package_path.glob(glob)]

setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    packages=[
        package_name,
        'anynet_disparity/AnyNet',
        'anynet_disparity/AnyNet/models'],
    data_files=[
        ('share/ament_index/resource_index/packages', 
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), ['config/checkpoint/kitti2015_ck/checkpoint.tar']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'anynet_disparity = anynet_disparity.anynet_disparity:main',
        ],
    },
)
