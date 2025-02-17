from setuptools import setup
import os
import urllib.request
import tarfile
from glob import glob

package_name = 'person_follower'

# Ruta donde se almacenará el vocabulario de ORB-SLAM3
vocab_dir = os.path.join('person_follower', 'ORB_SLAM3', 'Vocabulary')
vocab_file = os.path.join(vocab_dir, 'ORBvoc.txt')
compressed_file = os.path.join(vocab_dir, 'ORBvoc.txt.tar.gz')
download_url = 'https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/master/Vocabulary/ORBvoc.txt.tar.gz'

# Crear la carpeta si no existe
os.makedirs(vocab_dir, exist_ok=True)

# Descargar y extraer el archivo si no está presente
if not os.path.exists(vocab_file):
    print(f"📥 Descargando ORBvoc.txt.tar.gz desde {download_url}...")
    urllib.request.urlretrieve(download_url, compressed_file)
    print("✅ Descarga completada. Extrayendo archivo...")

    # Extraer el archivo
    try:
        with tarfile.open(compressed_file, "r:gz") as tar:
            tar.extractall(path=vocab_dir)
        print("✅ Extracción completada.")
    except Exception as e:
        print(f"❌ Error al extraer el archivo: {e}")
        exit(1)

    # Eliminar el archivo comprimido después de extraerlo
    os.remove(compressed_file)
    print("🗑️ Archivo comprimido eliminado para ahorrar espacio.")

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('person_follower/launch/*.launch.py')),
        (os.path.join('share', package_name, 'model'), glob('person_follower/model/*')),
        (os.path.join('share', package_name, 'config'), glob('person_follower/config/*.yaml')),
        (os.path.join('share', package_name, 'ORB_SLAM3/config'), glob('person_follower/ORB_SLAM3/config/*.yaml')),
        (os.path.join('share', package_name, 'ORB_SLAM3/Vocabulary'), [vocab_file]),  # Incluir el vocabulario extraído
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omixer',
    maintainer_email='al364109@uji.es',
    description='Package for person-following robot project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = person_follower.control_node.control_node:main',
            'camera_node = person_follower.camera_node.camera_node:main',
            'detection_node = person_follower.detection_node.detection_node:main',
            'tracking_node = person_follower.tracking_node.tracking_node:main',
            'collision_handling_node = person_follower.collision_handling_node.collision_handling_node:main',
            'user_interface_node = person_follower.user_interface_node.user_interface_node:main',
            'SLAM_node = person_follower.SLAM_node.SLAM_node:main',
        ],
    },
)
