from setuptools import find_packages, setup

package_name = 'dummy_hw_controller'

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
    maintainer='gofran',
    maintainer_email='zhanggaofan0827@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'dummy_hw = dummy_hw_controller.dummy_hw_controller.main'
            'dummy_hw = dummy_hw_controller.dummy_hw:main'
        ],
    },
)
