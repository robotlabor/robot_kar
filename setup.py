from setuptools import setup

package_name = 'robot_kar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arnl',
    maintainer_email='arnl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'bojarak2 = robot_kar.bojarak2:main',
        	'bojafel = robot_kar.bojafel:main'
        ],
    },
)
