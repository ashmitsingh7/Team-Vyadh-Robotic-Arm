from setuptools import find_packages, setup

package_name = 'arm_teleop'

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
    maintainer='Team Vyadh arm software (TODO: set maintainer)',
    maintainer_email='todo@example.com',
    description='Keyboard teleoperation: terminal input to /arm/user_input (UserInput intent).',
    license='TODO: select license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop_node = arm_teleop.keyboard_teleop_node:main',
        ],
    },
)
