from setuptools import find_packages, setup

package_name = 'move_with_obstacle'

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
    maintainer='maksim',
    maintainer_email='maksim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'test_zaicev = move_with_obstacle.test_zaicev:main',
        'obstacles = move_with_obstacle.obstacles:main',
        'obstacles_deepseek = move_with_obstacle.obstacles_deepseek:main',
        'obstacles_final = move_with_obstacle.obstacles_final:main'
        ],
    },
)
