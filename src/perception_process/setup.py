from setuptools import find_packages, setup

package_name = 'perception_process'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='pratyush',
    maintainer_email='msgpratyush@gmail.com',
    description='process published images',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone_estimate = perception_process.cone_estimate:main'
        ],
    },
)
