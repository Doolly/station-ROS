#!/usr/bin/python
#_*_coding:utf-8_*_

import rospy
from std_msgs.msg import Int8MultiArray, Int8, String, Bool

global floor
global state
global destination

class ItemStatusSubscriber():
    def __init__(self):
        self.item_status_sub = rospy.Subscriber("wstation/item_status", Int8MultiArray, callback=self._callback)
        self.item_status_buf = None

    def function(self):
        if self.item_status_buf is None:
            return None
        else:
            self.item_status_buf = None
            return True
        # print(self.item_status_buf)
    
    def _callback(self, msg):
        global floor
        matrix = [[0 for msg.layout.dim[0].stride in range(msg.layout.dim[0].size)] for msg.layout.dim[1].stride in range(msg.layout.dim[1].size)]
        floor = 1

        for j in range(msg.layout.dim[1].size):
            for i in range(msg.layout.dim[0].size):
                # j_multiply = j * msg.layout.dim[1].size
                matrix[j][i] = msg.data[i+j*(msg.layout.dim[1].size+1)]
                if j != msg.layout.dim[1].size - 1 and matrix[j][i] != 0:
                    floor = msg.layout.dim[1].size - j
        if floor != 0:
            self.item_status_buf = True
      

class LiftDestinationFloorPublisher():
    def __init__(self):
        self.lift_destination_floor_pub = rospy.Publisher("wstation/lift_destination_floor", Int8, queue_size=1)
        self.msg = Int8()

    def send_lift_destination_floor(self,floor):
        self.msg.data = floor
        self.lift_destination_floor_pub.publish(self.msg)
      
class LiftStatusSubscriber():
    def __init__(self):
        self.lift_status_sub = rospy.Subscriber("wstation/lift_status", String, callback=self._callback)
        self.lift_status_buf = None

    def function(self):
        return self.lift_status_buf
    
    def _callback(self, msg):
        self.lift_status_buf = msg.data



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


class LiftItemStatusSubscriber():
    def __init__(self):
        self.lift_item_status_sub = rospy.Subscriber("wstation/lift_item_status", String, callback=self._callback)
        self.lift_item_status_buf = None

    def function(self):        
        global state

        if self.lift_item_status_buf == True:
            self.lift_item_status_buf = None
            return True
        else:
            return None
    
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

    item_status_sub            = ItemStatusSubscriber()
    lift_destination_floor_pub = LiftDestinationFloorPublisher()
    lift_status_sub            = LiftStatusSubscriber()
    lift_current_floor_sub     = LiftCurrentFloorSubscriber()
    push_item_to_lift_pub     = PushItemToLiftPublisher()
    lift_item_status_sub       = LiftItemStatusSubscriber()
    lift_item_size_sub         = LiftItemSizeSubscriber()
    send_to_destination_pub    = SendToDestinationPublisher()

    state = 1
    destinate = -1
    push = "none"
    send_destination = "none"
    print("wstation start")

    while not rospy.is_shutdown():
        rate.sleep()
        
        # Publish
        lift_destination_floor_pub.send_lift_destination_floor(destinate)
        push_item_to_lift_pub.send_push_item_to_lift(push)
        send_to_destination_pub.send_to_destination(send_destination)


        # Check Lift Status and Compare Lift's current floor and Destination
        # to reset destination and command moving the lift
        if lift_status_sub.function() == "arrived" and lift_item_status_sub.function() == "none":
        #and lift_current_floor_sub.function() is destinate:
            print("Lift is reached at floor")
            destinate = -1
            push = "push"
            print("Push item to lift")


        # Check Lift Status and Convayer's Item Status
        # to Send Destination of Lift
        elif lift_status_sub.function() != "moving" and item_status_sub.function() is True:
            print("Item is in conveyor belt")
            destinate = floor
            print("Send Goal")



        elif lift_item_status_sub.function() == "exist":
            push = "none"

        # default is needcheck
        elif lift_item_status_sub.function() == True and lift_item_size_sub.function() == "good": 
            print("Box size is good to shipping")
            send_destination = "james"

        elif lift_item_status_sub.function() == True and lift_item_size_sub.function() == "bad" :
            print("Box is NOT good to shipping")
            send_destination = "tray"

        elif lift_item_status_sub.function() == False and send_destination != "none":
            send_destination = "none"


if __name__ == '__main__':
    try:
        wstation_main()
    except rospy.ROSInterruptException:
        pass