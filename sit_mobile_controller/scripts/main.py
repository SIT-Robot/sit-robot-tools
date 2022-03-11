import rospy
from std_msgs.msg import Bool

from basketball_hal import shovel,shoot

rospy.init_node('modile_agent')

m_shovel = shovel.Shovel()

def shovel_up_cb(b:Bool):
    if b.data:
        m_shovel.up()
    else:
        m_shovel.stop()
rospy.Subscriber('shovel_up',Bool,shovel_up_cb)

def shovel_down_cb(b:Bool):
    if b.data:
        m_shovel.down()
    else:
        m_shovel.stop()
rospy.Subscriber('shovel_down',Bool,shovel_down_cb)

m_shoot = shoot.Shoot()

def shoot_cb(b:Bool):
    if b.data:
        if m_shoot.status == shoot.ShootStatusEnum.READY:
            m_shoot.shoot()
rospy.Subscriber('shoot',Bool,shoot_cb)

def charge_cb(b:Bool):
    if b.data:
        m_shoot.charge(1)
rospy.Subscriber('charge',Bool,charge_cb)



try:
    rospy.spin()
except KeyboardInterrupt:
    pass

from queue import Queue

