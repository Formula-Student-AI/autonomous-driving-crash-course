from setuptools import setup

package_name = 'fsai_intro'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bristol FSAI',
    maintainer_email='noreply@example.com',
    description='Module 1 exercises: first ROS 2 nodes for the Bristol FSAI simulator',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ex1_listener = fsai_intro.ex1_listener:main',
            'ex2_driver = fsai_intro.ex2_driver:main',
            'ex3_mission = fsai_intro.ex3_mission:main',
            'ex4_speed_hold = fsai_intro.ex4_speed_hold:main',
            'ex5_nearest_cone = fsai_intro.ex5_nearest_cone:main',
        ],
    },
)
