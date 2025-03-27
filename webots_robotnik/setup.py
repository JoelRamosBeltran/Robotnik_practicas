from setuptools import setup

package_name = 'webots_robotnik'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch_beta.py','launch/robot_launch_avanzado.py']))
data_files.append(('share/' + package_name + '/worlds',['worlds/Prueba_rbrobout_webots.wbt']))
data_files.append(('share/' + package_name + '/protos',['protos/rbrobout.proto']))
data_files.append(('share/' + package_name + '/resource',['resource/rbrobout.urdf','resource/rbrobout_controller_params.yaml','resource/rbrobout_controller.urdf','resource/rbrobout_controller_avanzado.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joel',
    maintainer_email='joelramosbeltran@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = my_package.my_robot_driver:main'
        ],
    },
)
