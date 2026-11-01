from glob import glob
import os
from setuptools import setup


def tree(folder):
    """data_files for the regular files under folder, subfolders kept; Python caches skipped
    (launch files and scripts leave __pycache__ beside them, which setuptools cannot copy)."""
    files = {}
    for path in sorted(glob(folder+'/**/*', recursive=True)):
        if os.path.isfile(path) and '__pycache__' not in path.split(os.sep):
            files.setdefault('share/amr_lite_sim/'+os.path.dirname(path), []).append(path)
    return list(files.items())


setup(name='amr_lite_sim', version='0.1.0', packages=['amr_lite_sim'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/amr_lite_sim']),
                  ('share/amr_lite_sim', ['package.xml'])]+tree('launch')+tree('config')+tree('scripts'),
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True, maintainer='Gengo',
      maintainer_email='gengo@users.noreply.github.com',
      description='Gazebo-free lite simulation engine', license='Apache-2.0',
      entry_points={'console_scripts': ['lite_sim = amr_lite_sim.node:main', 'lite_survey = amr_lite_sim.survey:main', 'free_space_relay = amr_lite_sim.free_space_relay:main']})
