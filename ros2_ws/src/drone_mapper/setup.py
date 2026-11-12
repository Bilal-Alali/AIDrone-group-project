from setuptools import find_packages, setup

package_name = 'drone_mapper'

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
    maintainer='Drohnen-Team',
    maintainer_email='team@example.com',
    description='2D-Belegungskarte aus MAVLink-Abstandssensoren',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mapper_node = drone_mapper.mapper_node:main',
        ],
    },
)
