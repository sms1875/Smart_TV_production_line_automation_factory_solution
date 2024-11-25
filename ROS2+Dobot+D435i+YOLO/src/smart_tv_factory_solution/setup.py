from setuptools import find_packages, setup
from glob import glob

package_name = 'smart_tv_factory_solution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssafy',
    maintainer_email='67058185+sms1875@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_teaching=smart_tv_factory_solution.robot_teaching:main',
            'dobot_control_node=smart_tv_factory_solution.dobot_control_node:main',
            'detect_panel_node = smart_tv_factory_solution.detect_panel_node:main'
        ],
    },
)
