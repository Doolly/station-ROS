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
        # priority => high floor first
        if (self.secondFloorCount > self.thirdFloorCount)
            return 2
        elif (self.secondFloorCount < self.thirdFloorCount)
            return 3
        elif (self.secondFloorCount == 0 and self.thirdFloorCount == 0)
            return -1
        else
            # same count
            return 3
    
    def _callback(self, msg):
        self.secondFloorCount = 0
        self.thirdFloorCount = 0

        matrix = [[0 for msg.layout.dim[0].stride in range(msg.layout.dim[0].size)] for msg.layout.dim[1].stride in range(msg.layout.dim[1].size)]
        floor = 1

        for i in range(msg.layout.dim[0].size):
            for j in range(msg.layout.dim[1].size):
                matrix[i][j] = msg.data[(i * msg.layout.dim[1].size) + j]

        for i in range(msg.layout.dim[0].size)
            if (matrix[1][i] == 1)
                self.secondFloorCount += 1
            if (matrix[2][i] == 1)
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
        self.push_item_to_lift_pub = rospy.Publisher("wstation/push_item_to_lift", String, queue_size=1)
        self.msg = String()

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
        self.lift_item_state_sub = rospy.Subscriber("wstation/lift_item_size", String, callback=self._callback)
        self.lift_item_state_buf = None

    def function(self):        
        global state

        if self.lift_item_state_buf == "OK":
            self.lift_item_state_buf = None
            state = 7
            return state
        elif self.lift_item_state_buf == "TooBig":
            self.lift_item_state_buf = None
            state = 8
            return state
    
    def _callback(self, msg):
        self.lift_item_state_buf = msg.data


class SendToDestinationPublisher():
    def __init__(self):
        self.send_to_destination_pub = rospy.Publisher("wstation/send_to_destination", String, queue_size=1)
        self.msg = String()

    def send_to_destination(self, message):
        self.msg.data = message
        print("go to "+message+" is published")
        self.send_to_destination_pub.publish(self.msg)


        
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

    currentFloor = 0
    destinationFloor = 0

    liftStatus = ""
    liftItemStatus = ""
    itemSizeStatus = ""

    print("Wstation start")
    while not rospy.is_shutdown():
        rate.sleep()

        # Subscribe from Arduino
        currentFloor = lift_current_floor_sub.function()            # 1, 2, 3
        liftStatus = lift_status_sub.function()                     # "wait", "move", "arrived"
        liftItemStatus = lift_item_status_sub.function()            # "none", "exist"

        # ros wstation process
        if (liftStatus == "wait")
            print("Stage1 => Item is in the Conveyor Belt")
            destinationFloor = item_status_sub.function()

            if (destinationFloor != -1)
                lift_destination_floor_pub.send_lift_destination_floor(destinationFloor)

        elif (liftStatus == "arrived" and liftItemStatus == "none":
            print("State2 => Push Item")
            destinationFloor = -1
            push_item_to_lift_pub.send_push_item_to_lift("push")
            print("State2 => push item to lift")

        elif (liftStatus == "arrived" and liftItemStatus == "exist")
            print("State3 => Camera Check!")

            # ros 에서 우선 send_to_destination 메세지를 주기 전에 1층으로 움직이라는 명령을 주어야 함
            if (currentFloor != 1)
                lift_destination_floor_pub.send_lift_destination_floor(1)
                continue

            # item size check needed!
            # itemSizeStatus = lift_item_size_sub.function()
            itemSizeStatus = "bad"

            if (itemSizeStatus == "good")
                print("State4 => Send to James")

                if (currentFloor == 1)
                    send_to_destination_pub.send_to_destination("james")

            elif (itemSizeStatus == "bad")
                print("State5 => Send to Tray")

                if (currentFloor == 1)
                    send_to_destination_pub.send_to_destination("tray")

if __name__ == '__main__':
    try:
        wstation_main()
    except rospy.ROSInterruptException:
        pass