from setuptools import setup
import os
from glob import glob

package_name = 'control_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launchファイルをインストール
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        
        # CSVファイルをインストール
        (os.path.join('share', package_name, 'data'),
            glob('data/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hiro',
    maintainer_email='watanabe21@ctrl.fr.dendai.ac.jp',
    description='Motor controller test package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_test = control_test.controller_test:main',
        ],
    },
)