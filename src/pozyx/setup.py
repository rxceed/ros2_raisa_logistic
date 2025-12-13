from setuptools import setup

package_name = 'pozyx'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, 'pypozyx', 'pypozyx/structures', 'pypozyx/definitions', 'pypozyx/tools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='UWB Pozyx Pose2D Publisher',
    license='MIT',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwb_pub = pozyx.uwb_publisher:main',
            'uwb_logger = pozyx.uwb_logger:main'
        ],
    },
)
