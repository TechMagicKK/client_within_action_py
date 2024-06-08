from setuptools import find_packages, setup

package_name = 'client_within_action_py'

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
    maintainer='tajima',
    maintainer_email='tajima.ryosuke@techmagic.co.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client_within_action_1 = client_within_action_py.client_within_action_node_1:main',
            'client_within_action_2 = client_within_action_py.client_within_action_node_2:main',
            'client_within_action_3 = client_within_action_py.client_within_action_node_3:main',
            'client_within_action_4 = client_within_action_py.client_within_action_node_4:main',
            'client_within_action_5 = client_within_action_py.client_within_action_node_5:main',
            'client_within_action_6 = client_within_action_py.client_within_action_node_6:main',
            'periodic_action_client = client_within_action_py.periodic_action_client:main',
        ],
    },
)
