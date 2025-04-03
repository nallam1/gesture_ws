from setuptools import setup, find_packages

package_name = 'ads1263_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    include_package_data=True,
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nader',
    maintainer_email='nader.allam@mail.utoronto.ca',
    description='Reads from ADS1263 ADC and publishes sEMG data.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adc_node = ads1263_reader.semg_reader_node:main'
        ],
    },
)
