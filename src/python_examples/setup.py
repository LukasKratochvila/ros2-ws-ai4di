from setuptools import setup

package_name = 'python_examples'

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
    maintainer='Adam Ligocki',
    maintainer_email='adam.ligocki@vutbr.cz',
    description='Python examples for ROS2',
    license='Apache Licence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = python_examples.publisher:main',
            'subscriber = python_examples.subscriber:main',
            'service = python_examples.service:main',
            'client = python_examples.client:main',
            'action_server = python_examples.action_server:main',
            'action_client = python_examples.action_client:main',
        ],
    },
)
