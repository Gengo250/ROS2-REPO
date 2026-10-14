import importlib.util
from pathlib import Path
import pytest
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


@pytest.mark.parametrize('scenario', ['normal', 'person_crossing', 'obstacle_in_aisle', 'corridor_blocked'])
def test_composes_existing_headless_launch(scenario, monkeypatch):
    path = Path(__file__).resolve().parents[1] / 'launch/digital_twin.launch.py'
    spec = importlib.util.spec_from_file_location('twin_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entities = module.generate_launch_description().entities
    includes = [a for a in entities if isinstance(a, IncludeLaunchDescription)]
    assert len(includes) == 1
    context = LaunchContext()
    context.launch_configurations['scenario'] = scenario
    include = includes[0]
    args = {k: perform_substitutions(context, normalize_to_list_of_substitutions(v))
            for k, v in include.launch_arguments}
    assert args == {'scenario': scenario, 'gui': 'false', 'rviz': 'false'}
    assert not any(isinstance(a, SetEnvironmentVariable) for a in entities)
    include.launch_description_source.get_launch_description(context)
    assert include.launch_description_source.location.endswith('amr_simulation/launch/warehouse.launch.py')
