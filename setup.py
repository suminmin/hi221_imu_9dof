from setuptools import setup
import os
from glob import glob

package_name = 'hi221_imu_9dof'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'hi221_imu_9dof.imu_node',
        'hi221_imu_9dof.display_3D_visualization'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob("launch/*.launch.py")),
        (os.path.join('share', package_name), glob("config/*.yaml")),
        (os.path.join('share', package_name), glob("config/*.json")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = hi221_imu_9dof.imu_node:main',
            'display_3D_visualization = hi221_imu_9dof.display_3D_visualization:main',
        ],
    },
)
