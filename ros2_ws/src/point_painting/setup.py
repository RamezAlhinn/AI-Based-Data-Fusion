from setuptools import find_packages, setup

package_name = 'point_painting'

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
    maintainer='Ramez Alhinn',
    maintainer_email='ramez.hn5@gmail.com',
    description='PointPainting: fuse LiDAR points with camera segmentation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'painting_node = point_painting.painting_node:main',
        ],
    },
)
