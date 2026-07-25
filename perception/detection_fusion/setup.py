from setuptools import find_packages, setup
from common_python.setup_util import get_data_files

package_name = 'detection_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=get_data_files(package_name, ('launch', 'config')),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fuga Inagaki',
    maintainer_email='25fmr05@ms.dendai.ac.jp',
    description='Lane mask fusion node for AIFormula',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_fusion = detection_fusion.detection_fusion_node:main',
        ],
    },
)
