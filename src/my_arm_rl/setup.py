from setuptools import setup

package_name = 'my_arm_rl'

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
    maintainer='mark huang',
    maintainer_email='giraftw2002@gmail.com',
    description='RL training for 6DOF robot arm',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_rl = my_arm_rl.cld_train_rl:main',
            'test_rl = my_arm_rl.cld_test_rl:main',
        ],
    },
)