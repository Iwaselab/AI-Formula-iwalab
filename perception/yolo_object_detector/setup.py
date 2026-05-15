import os

from setuptools import find_packages, setup


def get_data_files(package_name, target_directories=()):
    data_files = [
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (os.path.join('share', package_name), ['package.xml']),
    ]
    for target_directory in target_directories:
        for root, _, files in os.walk(target_directory):
            for filename in files:
                file_path = os.path.join(root, filename)
                data_files.append((os.path.join('share', package_name, root), [file_path]))
    return data_files

package_name = 'yolo_object_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    install_requires=['ultralytics'],
    data_files=get_data_files(package_name, ('launch', 'config', 'resource')),
    zip_safe=True,
    maintainer='AI Formula',
    maintainer_email='devnull@example.com',
    description='YOLO-based object detection node for AI Formula',
    entry_points={
        'console_scripts': [
            'yolo_object_detector = yolo_object_detector.yolo_object_detector:main'
        ],
    },
)
