from setuptools import setup, find_packages

package_name = 'frustum_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/pipeline.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ramez Alhinn',
    maintainer_email='ramez.hn5@gmail.com',
    description='Frustum-based 3D object detection node for AI-Based-Data-Fusion.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frustum_node = frustum_detection.frustum_node:main',
            'metrics_logger_node = frustum_detection.metrics_logger_node:main',
            'bev_recorder_node = frustum_detection.bev_recorder_node:main',
        ],
    },
)
