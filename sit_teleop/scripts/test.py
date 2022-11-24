from dashboard import *


def test_draw_dashboard():
    info = ControlInfo()
    info.targetX = 1
    info.targetYawSpeed = -1
    drawingDashboard(info)


test_draw_dashboard()
