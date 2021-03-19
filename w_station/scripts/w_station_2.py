#!/usr/bin/python
#_*_coding:utf-8_*_

import rospy
from std_msgs.msg import Int8MultiArray, Int8, String, Bool

global floor
global state
global destination



class ItemStatusSubscriber():
    def __init__(self):
        self.item_status_sub = rospy.Subscriber("wstation/item_status", Int8MultiArray, callback = self._callback)
        self.item_status_buf = None

        self.secondFloorCount = 0
        self.thirdFloorCount = 0

    def function(self):
        # priority => high floor                                                    
        if (self.secondFloorCount > self.thirdFloorCount):
            return 2
        elif (self.secondFloorCount < self.thirdFloorCount):
            return 3
        elif (self.secondFloorCount == 0 and self.thirdFloorCount == 0):
            return -1
        else:
            # same count
            return 3
    
    def _callback(self, msg):
        self.secondFloorCount = 0
        self.thirdFloorCount = 0

        matrix = [[0 for msg.layout.dim[0].stride in range(msg.layout.dim[0].size)] for msg.layout.dim[1].stride in range(msg.layout.dim[1].size)]

        for i in range(msg.layout.dim[1].size):
            for j in range(msg.layout.dim[0].size):
                matrix[i][j] = msg.data[(i * msg.layout.dim[1].size) + j]

        for i in range(msg.layout.dim[1].size):
            if (matrix[1][i] == 1):
                self.secondFloorCount += 1
            if (matrix[2][i] == 1):
                self.thirdFloorCount += 1

class LiftDestinationFloorPublisher():
    def __init__(self):
        self.lift_destination_floor_pub = rospy.Publisher("wstation/lift_destination_floor", Int8, queue_size=1)
        self.msg = Int8()

    def send_lift_destination_floor(self,floor):
        self.msg.data = floor
        self.lift_destination_floor_pub.publish(self.msg)

class LiftCurrentFloorSubscriber():
    def __init__(self):
        self.lift_current_floor_sub = rospy.Subscriber("wstation/lift_current_floor", Int8, callback=self._callback)
        self.lift_current_floor_buf = None

    def function(self):
        return self.lift_current_floor_buf
    
    def _callback(self, msg):
        self.lift_current_floor_buf = msg.data


class PushItemToLiftPublisher():
    def __init__(self):
        self.push_item_to_lift_pub = rospy.Publisher("wstation/push_item_to_lift", Bool, queue_size=1)
        self.msg = Bool()

    def send_push_item_to_lift(self, push):
        self.msg.data = push
        self.push_item_to_lift_pub.publish(self.msg)

class LiftStatusSubscriber():
    def __init__(self):
        self.lift_status_sub = rospy.Subscriber("wstation/lift_status", String, callback=self._callback)
        self.lift_status_buf = None

    def function(self):
        return self.lift_status_buf

    def _callback(self, msg):
        self.lift_status_buf = msg.data

class LiftItemStatusSubscriber():
    def __init__(self):
        self.lift_item_status_sub = rospy.Subscriber("wstation/lift_item_status", String, callback=self._callback)
        self.lift_item_status_buf = None

    def function(self):
        return self.lift_item_status_buf
    
    def _callback(self, msg):
        self.lift_item_status_buf = msg.data


class LiftItemSizeSubscriber():
    def __init__(self):
        self.lift_item_size_sub = rospy.Subscriber("wstation/lift_item_size", String, callback=self._callback)
        self.lift_item_size_buf = None

    def function(self):        
        return self.lift_item_size_buf
    
    def _callback(self, msg):
        self.lift_item_size_buf = msg.data


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

    flooritemstatus = -1                # array, first 4 is 1st floor, and so on
    liftcurrentfloor = 0           # 1, 2, 3
    liftdestinationfloor = -1      # -1, 1, 2, 3
    liftstatus = "arrived"         # arrived, up, down
    liftitemstatus = False         # True, False
    liftitemsize = "none"          # good, bad, none
    pushitem = False               # True, False
    sendtodestination = "none"     # james, tray, none
    jamesarrived = False           # True, False # For identify James is Exist or not


class WstationTask(TopicList):
    def __init__(self):
        print("hi")
    
    # def get_topic(self,data1,data2,data3,data4,data5,data6,data7,data8,data9):
    
    def is_lift_has_item(self):
        if (liftitemstatus == True) and (jamesarrived == True):
            send_to_james()
        else:
            sendtodestination = "none"    

    def wait_james(self):
        sendtodestination = "james"
        

    def is_floor_has_item(self):
        if (liftitemstatus == False) and (liftcurrentfloor == 1):
            # need delay??
            if(itemstatus != -1):
                set_destination_floor()

    def set_destination_floor(self):
        liftdestinationfloor = itemstatus


    def is_lift_arrived(self):
        if (liftitemstatus == False) and (liftdestinationfloor == liftcurrentfloor):
            liftdestinationfloor = -1
            push_item_to_lift()

    def push_item_to_lift(self):
        pushitem = True
    

    def is_item_exist_in_lift(self):
        if (liftitemstatus == True):
            print("HI")

    # def is_item_push_into_lift(self):

    # def is_item_push_into_lift(self):
        

####################################################################################################################
####################################################################################################################

        
def wstation_main():
    global state, floor
    rospy.init_node('Wstation',anonymous=False)
    rate = rospy.Rate(4) # 300hz?

    lift_destination_floor_pub = LiftDestinationFloorPublisher()    # 1, 2, 3
    send_to_destination_pub    = SendToDestinationPublisher()       # "james", "tray"
    push_item_to_lift_pub      = PushItemToLiftPublisher()          # "push"

    item_status_sub            = ItemStatusSubscriber()             # IrSensor list
    lift_current_floor_sub     = LiftCurrentFloorSubscriber()
    lift_status_sub            = LiftStatusSubscriber()
    lift_item_status_sub       = LiftItemStatusSubscriber()
    lift_item_size_sub         = LiftItemSizeSubscriber()


    print("Wstation start")
    while not rospy.is_shutdown():

        rate.sleep()

        # Publish
        lift_destination_floor_pub.send_lift_destination_floor(TopicList.liftdestinationfloor)
        push_item_to_lift_pub.send_push_item_to_lift(TopicList.pushitem)

        # Subscribe from Arduino
        TopicList.liftcurrentfloor = lift_current_floor_sub.function()            # 1, 2, 3
        TopicList.liftstatus = lift_status_sub.function()                     # "wait", "move", "arrived"
        TopicList.liftitemstatus = lift_item_status_sub.function()            # "none", "exist"
        TopicList.liftitemsize = lift_item_size_sub.function()            # "good", "bad"



        WstationTask.is_lift_has_item()

        WstationTask.is_floor_has_item()

        WstationTask.is_lift_arrived()

        WstationTask.is_item_pushed_in_lift()


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