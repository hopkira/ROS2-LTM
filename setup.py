from setuptools import find_packages, setup

package_name = 'ltm_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Richard Hopkins',
    maintainer_email='hopkira@gmail.com',
    description='Long term RAG memory service for robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'service = ltm_package.service_ltm:main',
	'client = ltm_package.ltm_test_client:main',
        ],
    },
)