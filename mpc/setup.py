from setuptools import setup

package_name = 'mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
        'launch',
        'launch_ros',
        'rviz2',
        'geometry_msgs', 
        'tf2_ros', 
        ],
    zip_safe=True,
    maintainer='jooeon',
    maintainer_email='jooeonjohn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_recorder = py_pubsub.publisher_member_function:main',
            'csv_action_client_timer = py_pubsub.subscriber_member_function:main',
            'mpc_topic = py_pubsub.subscriber_member_function:main',
            'pid_topic = py_pubsub.subscriber_member_function:main',
            'time_dist_error = py_pubsub.subscriber_member_function:main'
        ],
    },
)
