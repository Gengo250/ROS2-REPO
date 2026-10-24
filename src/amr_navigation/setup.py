from glob import glob
from setuptools import setup
setup(name='amr_navigation', version='0.1.0', packages=['amr_navigation'],
      data_files=[('share/ament_index/resource_index/packages',['resource/amr_navigation']),
                  ('share/amr_navigation',['package.xml'])]+[
                      ('share/amr_navigation/'+folder,glob(folder+'/*'))
                      for folder in ['launch','config','maps','rviz','scripts']],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True, maintainer='Gengo',
      maintainer_email='gengo@users.noreply.github.com', description='Warehouse AMR navigation',
      license='Apache-2.0', entry_points={'console_scripts':[
          'sensor_audit = amr_navigation.audit:main',
          'initial_pose = amr_navigation.initial_pose:main',
          'command_guard = amr_navigation.command_guard:main',
          'navigate = amr_navigation.goal_client:main',
          'runtime_audit = amr_navigation.runtime_audit:main',
      ]})
