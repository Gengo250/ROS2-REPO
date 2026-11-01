from glob import glob
from setuptools import setup

setup(name='atlas_scene', version='0.1.0', packages=['atlas_scene'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/atlas_scene']),
                  ('share/atlas_scene', ['package.xml']),
                  ('share/atlas_scene/config', glob('config/*.json')),
                  ('share/atlas_scene/compiler', glob('compiler/*.mjs'))],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True, maintainer='Gengo',
      maintainer_email='gengo@users.noreply.github.com',
      description='Compiled ATLAS campus scene and geometry helpers', license='Apache-2.0')
