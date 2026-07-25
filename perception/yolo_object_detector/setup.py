from setuptools import find_packages, setup
from common_python.setup_util import get_data_files

package_name = 'yolo_object_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=get_data_files(package_name, ('launch', 'config', 'weights')),
    install_requires=[
        'ultralytics',
    ],
    zip_safe=True,
    maintainer='Fuga Inagaki',
    maintainer_email='25fmr05@ms.dendai.ac.jp',
    description='YOLO-based object detection node for AI-Formula',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_object_detector = yolo_object_detector.yolo_object_detector:main'
        ],
    },
)
