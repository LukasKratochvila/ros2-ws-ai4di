from setuptools import setup

package_name = 'img_pub_py'

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
            'img_pub = img_pub_py.img_pub_py:main',
            'file_pub = img_pub_py.file_pub:main',
        ],
    },
)
