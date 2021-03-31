#!/usr/bin/python
#_*_coding:utf-8_*_

import rospy
from std_msgs.msg import Int8, String, Bool

class LiftDestinationFloorPublisher():
    def __init__(self):
        self.lift_destination_floor_pub = rospy.Publisher("wstation/lift_destination_floor", Int8, queue_size=1)
        self.msg = Int8()

    def send_lift_destination_floor(self,floor):
        self.msg.data = floor
        self.lift_destination_floor_pub.publish(self.msg)


class PushItemToLiftPublisher():
    def __init__(self):
        self.push_item_to_lift_pub = rospy.Publisher("wstation/push_item", Bool, queue_size=1)
        self.msg = Bool()

    def send_push_item_to_lift(self, push):
        self.msg.data = push
        self.push_item_to_lift_pub.publish(self.msg)


class SendToDestinationPublisher():
    def __init__(self):
        self.send_to_destination_pub = rospy.Publisher("wstation/send_to_destination", String, queue_size=1)
        self.msg = String()

    def send_to_destination(self, message):
        self.msg.data = message
        self.send_to_destination_pub.publish(self.msg)



######################################################################################################################
######################################################################################################################

class TopicList:

    liftdestinationfloor = -1      # -1, 1, 2, 3
    sendtodestination = "none"     # james, tray, none
    pushitem = False               # True, False
    status = "none"
    

class WstationStatusSubscriber(TopicList):
    def __init__(self):
        self.wstation_status_sub = rospy.Subscriber("wstation/status", String, callback=self._callback)
        self.wstation_status_buf = self.status

    # waitjames,tojames,totray,gofirstfloor,gosecondfloor,gothirdfloor,pushitem,waititem
    def function(self):
        
            
        if self.wstation_status_buf == "waitjames":
            TopicList.liftdestinationfloor = -1
            TopicList.sendtodestination = "none"
            TopicList.pushitem = False 

        elif self.wstation_status_buf == "tojames":
            TopicList.liftdestinationfloor = -1
            TopicList.sendtodestination = "james"
            TopicList.pushitem = False

        elif self.wstation_status_buf == "totray":
            TopicList.liftdestinationfloor = -1
            TopicList.sendtodestination = "tray"
            TopicList.pushitem = False 

        elif self.wstation_status_buf == "gofirstfloor":
            TopicList.liftdestinationfloor = 1
            TopicList.sendtodestination = "none"
            TopicList.pushitem = False 

        elif self.wstation_status_buf == "gosecondfloor":
            TopicList.liftdestinationfloor = 2
            TopicList.sendtodestination = "none"
            TopicList.pushitem = False 

        elif self.wstation_status_buf == "gothirdfloor":
            TopicList.liftdestinationfloor = 3
            TopicList.sendtodestination = "none"
            TopicList.pushitem = False 

        elif self.wstation_status_buf == "pushitem":
            TopicList.liftdestinationfloor = -1
            TopicList.sendtodestination = "none"
            TopicList.pushitem = True
        
        else: #
            TopicList.liftdestinationfloor = -1
            TopicList.sendtodestination = "none"
            TopicList.pushitem = False

        # print(self.wstation_status_buf)
        # print(TopicList.liftdestinationfloor)
        # print(TopicList.sendtodestination)
        # print(TopicList.pushitem)
    
    def _callback(self, msg):
        self.wstation_status_buf = msg.data


####################################################################################################################
####################################################################################################################

        
def wstation_main():

    rospy.init_node('Wstation_pub',anonymous=False)
    rate = rospy.Rate(10)

    lift_destination_floor_pub = LiftDestinationFloorPublisher()    # -1, 1, 2, 3
    send_to_destination_pub    = SendToDestinationPublisher()       # "james", "tray", "none"
    push_item_to_lift_pub      = PushItemToLiftPublisher()          # "push" ,"none"
    wstation_status_sub        = WstationStatusSubscriber()


    print("Wstation start")
    while not rospy.is_shutdown():

        rate.sleep()

        wstation_status_sub.function()

        # Publish
        if (TopicList.status != "emergency") or (TopicList.status != "manual"):
            lift_destination_floor_pub.send_lift_destination_floor(TopicList.liftdestinationfloor)
            send_to_destination_pub.send_to_destination(TopicList.sendtodestination)
            push_item_to_lift_pub.send_push_item_to_lift(TopicList.pushitem)
        print("liftdest : "+str(TopicList.liftdestinationfloor)+" send : "+TopicList.sendtodestination+" pushitem : "+str(TopicList.pushitem))

if __name__ == '__main__':
    try:
        wstation_main()
    except rospy.ROSInterruptException:
        pass