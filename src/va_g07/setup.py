from glob import glob

from setuptools import setup

package_name = 'va_g07'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rotas', glob('rotas/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grupo 07',
    maintainer_email='grupo07@ua.pt',
    description='Grupo 07: rota global GNSS -> objetivo local; simulador e perceção do modelo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fake_scan = va_g07.fake_scan:main',
            'scan_to_grid = va_g07.scan_to_grid:main',
            'rota_objetivo = va_g07.rota_objetivo:main',
            'rota_de_bag = va_g07.rota_de_bag:main',
        ],
    },
)
