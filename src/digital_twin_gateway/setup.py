from glob import glob
from setuptools import find_packages, setup

setup(
    name='digital_twin_gateway', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/digital_twin_gateway']),
        ('share/digital_twin_gateway', ['package.xml', 'README.md', 'requirements.txt']),
        ('share/digital_twin_gateway/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'anyio>=4,<5', 'fastapi>=0.115,<1', 'pydantic>=2.10,<3',
                      'uvicorn>=0.30,<1', 'websockets>=14,<16'],
    tests_require=['pytest', 'pytest-asyncio', 'httpx', 'jsonschema'],
    zip_safe=True, maintainer='gengo', maintainer_email='gengo@example.com',
    description='Read-only ROS telemetry gateway', license='Apache-2.0',
    entry_points={'console_scripts': ['gateway = digital_twin_gateway.main:main',
        'simulation_pose_adapter = digital_twin_gateway.ros.simulation_adapter:main']},
)
