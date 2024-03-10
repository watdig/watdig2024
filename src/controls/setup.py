"""
Setup file for Controls Package.
"""

from setuptools import find_packages, setup

PACKAGE_NAME = 'controls'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jivankesan',
    maintainer_email='jkesan28@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [\
            "controls_node = controls.controller_node:main",
            "action_server = scripts.turn_and_move_action_server:main"
        ],
    },
)
