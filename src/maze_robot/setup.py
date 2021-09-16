from setuptools import setup

package_name = 'maze_robot'

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
    maintainer='dawid',
    maintainer_email='dawid@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_controller = maze_robot.robot_controller:main",
            "camera = maze_robot.camera:main",
            "configure_camera = maze_robot.configure_camera:main",
            "camera_sim = maze_robot.camera_sim:main",
        ],
    },
)
