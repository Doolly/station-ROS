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
        self.push_item_to_lift_pub = rospy.Publisher("wstation/push_item_to_lift", Bool, queue_size=1)
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
        print("go to "+message+" is published")
        self.send_to_destination_pub.publish(self.msg)



######################################################################################################################
######################################################################################################################

class TopicList:

    liftdestinationfloor = -1      # -1, 1, 2, 3
    sendtodestination = "none"     # james, tray, none
    pushitem = False               # True, False
    

class WstationStatusSubscriber(TopicList):
    def __init__(self):
        self.wstation_status_sub = rospy.Subscriber("wstation/status", String, callback=self._callback)
        self.wstation_status_pub = None

    # waitjames,tojames,totray,gofirstfloor,gosecondfloor,gothirdfloor,pushitem
    def function(self):
        if status = "waitjames":
            liftdestinationfloor = -1
            sendtodestination = "none"
            pushitem = False 

        if status = "tojames":
            liftdestinationfloor = -1
            sendtodestination = "none"
            pushitem = False

        if status = "totray":
            liftdestinationfloor = -1
            sendtodestination = "none"
            pushitem = False 

        if status = "gofirstfloor":
            liftdestinationfloor = -1
            sendtodestination = "none"
            pushitem = False 

        if status = "gosecondfloor":
            liftdestinationfloor = -1
            sendtodestination = "none"
            pushitem = False 

        if status = "gothirdfloor":
            liftdestinationfloor = -1
            sendtodestination = "none"
            pushitem = False 

        if status = "pushitem":
            liftdestinationfloor = -1
            sendtodestination = "none"
            pushitem = False 
    
    def _callback(self, msg):
        self.lift_current_floor_buf = msg.data

    # def is_item_push_into_lift(self):

    # def is_item_push_into_lift(self):
        

####################################################################################################################
####################################################################################################################

        
def wstation_main():

    rospy.init_node('Wstation_pub',anonymous=False)
    rate = rospy.Rate(4) # 300hz?

    lift_destination_floor_pub = LiftDestinationFloorPublisher()    # -1, 1, 2, 3
    send_to_destination_pub    = SendToDestinationPublisher()       # "james", "tray", "none"
    push_item_to_lift_pub      = PushItemToLiftPublisher()          # "push" ,"none"
    wstation_status_sub        = WstationStatusSubscriber()


    print("Wstation start")
    while not rospy.is_shutdown():

        rate.sleep()

        wstation_status_sub.function()

        # Publish
        lift_destination_floor_pub.send_lift_destination_floor(TopicList.liftdestinationfloor)
        send_to_destination_pub.send_to_destination(TopicList.sendtodestination)
        push_item_to_lift_pub.send_push_item_to_lift(TopicList.pushitem)


        # if (liftStatus == "wait"):
        #     # print(TopicList.pushitem)
        #     TopicList.pushitem = True
            # print(TopicList.pushitem)

        # # ros wstation process
        # if (liftStatus == "wait"):
        #     print("Stage1 => start!")
        #     destinationFloor = item_status_sub.function()

        #     if (destinationFloor != -1):
        #         print("item is not exist")

        # elif (liftStatus == "arrived" and liftItemStatus == "none"):
        #     print("State2 => Push Item")

        #     destinationFloor = -1
        #     pushCommand = "push"
            
        #     print("State2 => push item to lift")

        # elif (liftStatus == "arrived" and liftItemStatus == "exist" and itemSizeStatus != "good" and itemSizeStatus != "bad"):
        #     print("State3 => Camera Check!")

        #     pushCommand = "none"

        #     # ros 에서 우선 send_to_destination 메세지를 주기 전에 1층으로 움직이라는 명령을 주어야 함
        #     if (currentFloor != 1):
        #         destinationFloor = 1
        #         continue
        #     else:
        #         destinationFloor = -1
        #     # item size check needed!
        #     # itemSizeStatus = lift_item_size_sub.function()
        #     # itemSizeStatus = "bad"

            

        # if (itemSizeStatus == "good"):
        #     print("State4 => Send to James")
        
        #     if (currentFloor == 1):
        #         send_to_destination_pub.send_to_destination("james")


        # elif (itemSizeStatus == "bad"):
        #     print("State5 => Send to Tray")

        #     if (currentFloor == 1):
        #         send_to_destination_pub.send_to_destination("tray")


if __name__ == '__main__':
    try:
        wstation_main()
    except rospy.ROSInterruptException:
        pass