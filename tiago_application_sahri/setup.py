from setuptools import find_packages, setup

package_name = 'tiago_application_sahri'

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
    maintainer='bal',
    maintainer_email='singhbal.baljinder@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
            'console_scripts': [
                    'planner = tiago_application_sahri.planner:main',
            ],
    },
)
