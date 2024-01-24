"""
Setup file for Communications Package.
"""

from setuptools import find_packages, setup

PACKAGE_NAME = 'communication'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/resource', ['resource/checkpoints.csv', 'resource/environment.csv', 'resource/obstacles.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jivankesan',
    maintainer_email='jkesan28@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            'position_client = communication.position_client:main',
            'csv_parse = communication.csv_parse:main'
        ],
    },
)
