from setuptools import setup

package_name = 'pose2car'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='remobility',
    maintainer_email='valentin.schaeffer@stud.hs-coburg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'posetocar = pose2car.pose2car:main',
            'maptocar = pose2car.map2car:main',
            'marker  = pose2car.marker:main'
        ],
    },
)
