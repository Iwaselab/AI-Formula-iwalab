from setuptools import find_packages, setup
from common_python.setup_util import get_data_files

package_name = 'lane_tracer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=get_data_files(package_name, ("config", "launch")),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aiformula',
    maintainer_email='noreply@example.com',
    description='Simple lane-following node (PointCloud2 -> Twist)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_tracer=lane_tracer.lane_tracer:main'
        ],
    },
)
