import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur_handeye_app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config (YAML) files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'data'), glob('data/**/*.*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iaslab',
    maintainer_email='iaslab@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
            'console_scripts': [
                    'hello = ur_handeye_app.hello:main',
                    'test_moveit = ur_handeye_app.test_moveit:main',
                    'data_acquisition = ur_handeye_app.handeye_data_acquisition:main',
                    'data_control = ur_handeye_app.handeye_data_control:main',
            ],
    },
)
