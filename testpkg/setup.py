from setuptools import setup

package_name = 'testpkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/testpkg.launch.py']), #launchファイル作ったら追加
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fuga',
    maintainer_email='fuga@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'testnode = testpkg.testnode:main',
            'testnode2 = testpkg.testnode2:main' #ノード作ったら追加
        ],
    },
)
