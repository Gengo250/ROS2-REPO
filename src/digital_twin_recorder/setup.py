from setuptools import find_packages, setup
from glob import glob
setup(name='digital_twin_recorder', version='0.2.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages',['resource/digital_twin_recorder']),
                ('share/digital_twin_recorder',['package.xml','README.md','requirements.txt']),
                ('share/digital_twin_recorder/launch',glob('launch/*.launch.py'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Gengo', maintainer_email='gengo@example.com', description='Independent ROS telemetry persistence via Mosaico',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts':['recorder = digital_twin_recorder.ros.recorder_node:main',
                                    'verify_persistence = digital_twin_recorder.storage.retrieval:main']})
