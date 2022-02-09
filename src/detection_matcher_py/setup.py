from setuptools import setup

package_name = 'detection_matcher_py'

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
    maintainer='kratochvila',
    maintainer_email='kratochvila.lukas@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_matcher_node = detection_matcher_py.detection_matcher_node:main',
            'detection_matcher_tester_node = detection_matcher_py.tester_node:main'
        ],
    },
)
