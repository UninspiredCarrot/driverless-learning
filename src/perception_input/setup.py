from setuptools import find_packages, setup

package_name = 'perception_input'

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
    maintainer='pratyush',
    maintainer_email='msgpratyush@gmail.com',
    description='receive input from perception hardware and publish',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed_camera_rectify = perception_input.zed_camera_rectify:main',
            'video_simulate = perception_input.video_simulate:main'
        ],
    },
)
