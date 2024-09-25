from setuptools import find_packages, setup

package_name = 'warmup_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mia',
    maintainer_email='mchevere@olin.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = warmup_project.teleop:main',
            'drive_square = warmup_project.drive_square:main',
            'obstacle_avoidance = warmup_project.obstacle_avoidance:main',
            'finite_state_controller = warmup_project.finite_state_controller:main',
            'wall_follower = warmup_project.wall_follower:main',
            'person_follower = warmup_project.person_follower:main',
        ],
    },
)
