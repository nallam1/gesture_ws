from setuptools import setup

package_name = 'ads1263_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nader',
    maintainer_email='nader.allam@mail.utoronto.ca',
    description='Reads sEMG data and publishes features',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semg_reader_node = ads1263_reader.semg_reader_node:main',
        ],
    },
)
