from setuptools import setup

package_name = 'cafe_butler'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pranava Swaroopa',
    maintainer_email='ppswaroopa@gmail.com',
    description='Cafe butler that delivers orders from Kitchen to Tables',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cafe_butler = cafe_butler.butler:main',
            'order_simulator = cafe_butler.orders:main'
        ],
    },
)
