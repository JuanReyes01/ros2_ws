from setuptools import setup

package_name = 'my_taller1'

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
    maintainer='yesid',
    maintainer_email='y.almanza@uniandes.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nodo1 = my_taller1.nodo1:main',
            'turtle_bot_teleop=my_taller1.keyboard_teleop_hold:main',
            'client_p1=my_taller1.MiCliente:main',
            'serviciot_p1=my_taller1.Servicio:main',
            'laser_info=my_taller1.laser_information:main',
            'laser_visualization=my_taller1.laser_visualization:main'

        ],
    },
)
