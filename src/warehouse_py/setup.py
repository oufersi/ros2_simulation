from setuptools import find_packages, setup

package_name = 'warehouse_py'

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
    maintainer='yangjingyun',
    maintainer_email='yangjingyun@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = warehouse_py.publisher_member_function:main',
            'listener = warehouse_py.subscriber_member_function:main',
            'joint_trajectory = warehouse_py.joint_trajectory_publisher:main',
            'diff_drive = warehouse_py.diff_drive:main',
            'test = warehouse_py.test:main',
        ],
    },
)
