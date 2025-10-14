from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'prl_ur5_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Fichiers standards requis par ROS 2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Installation du répertoire de lancement (launch)
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        
        # --- C'EST LA PARTIE CORRIGÉE ---
        # Installation de tous les fichiers du répertoire files/models
        # La destination est 'share/prl_ur5_calibration/files/models'
        # La source est tous les fichiers (*.*) dans 'files/models'
        (os.path.join('share', package_name, 'files', 'models'), glob(os.path.join('files', 'models', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Etienne Arlaud',
    maintainer_email='etienne.arlaud@inria.fr',
    description='This package contains the calibration nodes for the PRL UR5 dual arm setup',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate_external_camera = prl_ur5_calibration.calibrate_external_camera:main',
        ],
    },
)