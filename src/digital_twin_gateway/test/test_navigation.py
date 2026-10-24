from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from digital_twin_gateway.state.models import NavigationStatus

def test_navigation_is_separate_versioned_and_ages(state):
    manager,timer=state
    before=manager.publish().model_dump()
    assert manager.navigation_snapshot().diagnostics==()
    manager.update_navigation(DiagnosticArray(status=[DiagnosticStatus(name='navigation/interlock',level=0,message='ready',values=[KeyValue(key='localization_source',value='AMCL')])]))
    result=manager.navigation_snapshot()
    assert isinstance(result,NavigationStatus) and result.protocol_version=='1.0'
    assert result.diagnostics[0].availability=='online'
    timer.value+=manager.settings.stale_seconds+.1
    assert manager.navigation_snapshot().diagnostics[0].availability=='stale'
    assert manager.snapshot().model_dump()==before
    manager.update_navigation(DiagnosticArray(status=[DiagnosticStatus(name='unrelated',level=0,message='ignored')]))
    assert len(manager.navigation_snapshot().diagnostics)==1
