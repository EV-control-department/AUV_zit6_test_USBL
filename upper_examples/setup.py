from setuptools import find_packages, setup

package_name = 'upper_examples'

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
    maintainer='doc049',
    maintainer_email='zhangyinrui00@gmail.com',
    description='zit6通信例程',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_setter = upper_examples.pidset:main',
            'pid_getter = upper_examples.pidget:main',
        ],
    },
)
