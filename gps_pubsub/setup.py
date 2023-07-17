from setuptools import setup
import os
from glob import glob

package_name = 'gps_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+ package_name, ['data/gps_data/gps_data.csv']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mnc',
    maintainer_email='madhunc117@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_gps_data = gps_pubsub.gps_publisher:main',
            'subscribe_gps_data = gps_pubsub.gps_subscriber:main',
        ],
    },
)
