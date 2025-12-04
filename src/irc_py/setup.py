from setuptools import find_packages, setup

package_name = 'irc_py'

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
    maintainer='brinex',
    maintainer_email='brinex@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'irc_ps4 = irc_py.irc_ps4:main',
            'irc_arm = irc_py.irc_arm:main',
            'irc_joy = irc_py.irc_joy:main'
        ],
    },
)
