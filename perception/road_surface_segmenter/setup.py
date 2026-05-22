from setuptools import find_packages, setup
from common_python.setup_util import get_data_files

package_name = 'road_surface_segmenter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=get_data_files(package_name, ("launch", "weights", "config")),
    install_requires=[
        'setuptools',
        'torch>=1.7.0',
        'torchvision>=0.8.0',
        'numpy>=1.18.5',
        'opencv-python>=4.1.1',
    ],
    zip_safe=True,
    maintainer='masayaokada',
    maintainer_email='masayaokada@jp.honda',
    description='Road surface semantic segmentation using YOLOPv2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'road_surface_segmenter=road_surface_segmenter.road_surface_segmenter:main'
        ],
    },
)
