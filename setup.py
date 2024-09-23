from setuptools import setup

package_name = 'tree_publisher'

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
    maintainer='ros',
    maintainer_email='ruiiwata837@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = tree_publisher.test_publisher:main',
            'tree_publisher = tree_publisher.tree_publisher:main',
            'test_publisher = tree_publisher.tree_test:main',
        ],
    },
)
