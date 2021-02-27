#!/usr/bin/python
#_*_coding:utf-8_*_

import rospy
from std_msgs.msg import Int8MultiArray, Int8, String, Bool

global floor
global state


class ItemIsExistSubscriber():
    def __init__(self):
        self.item_is_exist_sub = rospy.Subscriber("wstation/item_is_exist", Int8MultiArray, callback=self._callback)
        self.item_is_exist_buf = None

    def function(self):
        if self.item_is_exist_buf is None:
            return None
        else:
            self.item_is_exist_buf = None
            return True
        # print(self.item_is_exist_buf)
    
    def _callback(self, msg):
        global floor
        matrix = [[0 for msg.layout.dim[0].stride in range(msg.layout.dim[0].size)] for msg.layout.dim[1].stride in range(msg.layout.dim[1].size)]
        floor = 2
        for j in range(msg.layout.dim[1].size):
            for i in range(msg.layout.dim[0].size):
                # j_multiply = j * msg.layout.dim[1].size
                matrix[j][i] = msg.data[i+j*(msg.layout.dim[1].size+1)]
                if j != 2 and matrix[j][i] != 0:
                    floor = 3 - j
        if floor != 0:
            self.item_is_exist_buf = True
      

class LiftGoalPublisher():
    def __init__(self):
        self.lift_goal_pub = rospy.Publisher("wstation/lift_goal", Int8, queue_size=1)
        self.msg = Int8()

    def send_lift_goal(self):
        global floor, state
        self.msg.data = floor
        self.lift_goal_pub.publish(self.msg)
        state = 3
        return state
      

class LiftPosStateSubscriber():
    def __init__(self):
        self.lift_pos_state_sub = rospy.Subscriber("wstation/lift_pos_state", Int8, callback=self._callback)
        self.lift_pos_state_buf = None

    def function(self):
        global floor, state
        if floor == self.lift_pos_state_buf:
            self.lift_pos_state_buf = None
            state = 4
            return True
        else:
            return None
    
    def _callback(self, msg):
        self.lift_pos_state_buf = msg.data


class PushItemToLiftPublisher():
    def __init__(self):
        self.push_item_to_lift_pub = rospy.Publisher("wstation/push_item_to_lift", Bool, queue_size=1)
        self.msg = Bool()

    def send_push_item_to_lift(self, _bool):
        global state
        if _bool is True:
            self.msg.data = _bool
            self.push_item_to_lift_pub.publish(self.msg)
            state = 5


class LiftItemStateSubscriber():
    def __init__(self):
        self.lift_item_state_sub = rospy.Subscriber("wstation/lift_item_state", Bool, callback=self._callback)
        self.lift_item_state_buf = None

    def function(self):        
        global state

        if self.lift_item_state_buf == True:
            self.lift_item_state_buf = None
            return True
        else:
            return None
    
    def _callback(self, msg):
        self.lift_item_state_buf = msg.data


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
    global state
    rospy.init_node('Wstation',anonymous=False)
    rate = rospy.Rate(3) # 300hz?

    item_is_exist_sub = ItemIsExistSubscriber()
    lift_goal_pub = LiftGoalPublisher()
    lift_pos_state_sub = LiftPosStateSubscriber()
    push_item_pub = PushItemToLiftPublisher()
    lift_item_state_sub = LiftItemStateSubscriber()
    lift_item_size_sub = LiftItemSizeSubscriber()
    send_to_destination_pub = SendToDestinationPublisher()

    state = 1
    print("wstation start")
    while not rospy.is_shutdown():
        rate.sleep()

        if state == 1:
            # print(item_is_exist_sub.function())
            if item_is_exist_sub.function() is True:
                print("Item is in conveyor belt")
                state = 2

        elif state == 2:
            lift_goal_pub.send_lift_goal()
            print("Send Goal")

        elif state == 3:
            if lift_pos_state_sub.function() is True:
                print("Lift is reached at floor")

        elif state == 4:
            push_item_pub.send_push_item_to_lift(True)
            print("Push item to lift")

        elif state == 5:
            if lift_item_state_sub.function() == True:
                print("Box is in the lift")
                state = 6
            else:
                print("Box is NOT in the lift")

        elif state == 6:
            if lift_item_size_sub.function() == 7:
                print("Box size is good to shipping")
            elif lift_item_size_sub.function() == 8:
                print("Box is NOT good to shipping")

        elif state == 7:
            send_to_destination_pub.send_to_destination("james")

        elif state == 8:
            send_to_destination_pub.send_to_destination("tray")

if __name__ == '__main__':
    try:
        wstation_main()
    except rospy.ROSInterruptException:
        pass