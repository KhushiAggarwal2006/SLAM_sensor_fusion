import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ekf_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name), ['rqtplugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='experiqs',
    maintainer_email='somnath.buriuly@experiqs.tech',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_estimator = ekf_1.node_1:main', #without VO data :)
            #'visualiser = ekf_1.visualiser:main',

        ],
    },
)
