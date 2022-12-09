#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


coord = {
    "A": ((5, 0), (-3.5, -3.5)),
    "B": ((5, -3.5), (-3.5, -7)),
    "C": ((5, -7), (-3.5, -14)),
    "D": ((10.5, 0), (5, -14)),
}

# coord = {
#     "A": ((5.5, 0), (-3, -3)),
#     "B": ((5.5, -3), (-3, -6.5)),
#     "C": ((5.5, -6.5), (-3, -14)),
#     "D": ((10.5, 0), (5.5, -14)),
# }

class Localization:
    def __init__(self):
        self.publisher = rospy.Publisher("/area", String, queue_size=1)
        self.subscriber = rospy.Subscriber("/slam_out_pose", PoseStamped, self.callback, queue_size = 1)
        self.area = "START"

    def callback(self, msg_in):
        x = msg_in.pose.position.x
        y = msg_in.pose.position.y
        curArea = self.area

        for a, coor in coord.items():
            if self.areaCheck(x, y, coor):
                curArea = a
                break

        if curArea != self.area:
            self.area = curArea
            self.publisher.publish(curArea)
            print(x , y)
            print("I AM AT", curArea)
            

    def areaCheck(self, x, y, coor):
        return coor[0][0] >= x and coor[0][1] >= y and x >= coor[1][0] and y >= coor[1][1]

def main():
    rospy.init_node("localization")
    node = Localization()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupted")

if __name__ == '__main__':
    main()
