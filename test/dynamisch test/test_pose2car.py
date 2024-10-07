import sys
sys.path.append('/home/rishi/icp_ws/src/map_und_pos/test')
from marvelmind_interfaces.msg import HedgePos, HedgeImuFusion
import testable_pose2car
from testable_pose2car import pubsub

marker = pubsub()


# Testfall 1: Überprüfung der Position auf dem Karte auf x und y
def test_position():
    msg = HedgePos()
    msg.x_m = 1.0
    msg.y_m = 1.0
    msg.z_m = 0.4

    
    output = marker.update_marker(msg)
    assert output[0] == 1.0
    assert output[1] == 1.0

# Testfall 2: Überprüfung der Orientierung auf dem Karte mit z 
def test_orientation():
    msg = HedgeImuFusion
    msg.x_m = 1.0
    msg.y_m = 1.0
    msg.z_m = 1.0

    
    output = marker.update_marker(msg)
    assert output[1] == 1.0
