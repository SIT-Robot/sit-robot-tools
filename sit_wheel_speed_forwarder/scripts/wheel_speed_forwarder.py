import rospy
import sys

from std_msgs.msg import Int16MultiArray

from sit_protocol_msgs.msg import DataFrame
from sit_protocol_msgs.srv import RequestDataRequest, RequestDataResponse, RequestData
from typing import *

rospy.init_node('wheel_speed_forwarder', sys.argv)

speed_pubber1 = rospy.Publisher('/wheels_speed', Int16MultiArray,queue_size=10)

req_proxy = rospy.ServiceProxy('/protocol_forwarder/RequestData', RequestData)
rospy.loginfo('等待串口服务器可用')
req_proxy.wait_for_service()
rospy.loginfo('串口服务器链接成功')


def get_speed() -> List[int]:
    req = RequestDataRequest()
    req.request.cmd = 0xa1
    req.waitCmd = 0x1a
    try:
        resp: RequestDataResponse = req_proxy.call(req)
        bs = resp.response.data

        return list(
            map(lambda x: ((bs[2*x] << 8) | bs[2*x+1]) - 5000,
                range(0, 4)))

    except rospy.ServiceException as e:
        rospy.logerr('请求超时: '+str(e))


def timer(t):
    s = get_speed()
    if s is not None:
        ima = Int16MultiArray()
        ima.data = s
        speed_pubber1.publish(ima)


rospy.Timer(rospy.Duration.from_sec(0.01), timer)

try:
    rospy.spin()
except KeyboardInterrupt:
    pass
