from glob import glob
from pathlib import Path
from setuptools import setup
setup(name='amr_route_planner', version='0.1.0', packages=['amr_route_planner'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/amr_route_planner']),
                  ('share/amr_route_planner', ['package.xml'])]+[
                      ('share/amr_route_planner/'+folder, [f for f in glob(folder+'/*') if Path(f).is_file()])
                      for folder in ['config', 'scripts']],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True, maintainer='Gengo',
      maintainer_email='gengo@users.noreply.github.com', description='Warehouse graph route planning',
      license='Apache-2.0', entry_points={'console_scripts': [
          'build_graph = amr_route_planner.cli:build_graph',
          'calibrate_registration = amr_route_planner.cli:calibrate',
          'plan_route = amr_route_planner.cli:plan',
          'execute_route = amr_route_planner.route_executor:main',
      ]})
