#!/usr/bin/python
#_*_coding:utf-8_*_

import rospy
from std_msgs.msg import Int8MultiArray, Int8, String, Bool

global floor
global state
global destination



class FloorItemStatusSubscriber():
    def __init__(self):
        self.floor_item_status_sub = rospy.Subscriber("wstation/item_status", Int8MultiArray, callback = self._callback)
        self.floor_item_status_buf = None

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

        sizeofarray = 5
        numofarray = 3

        matrix = [[0 for _ in range(sizeofarray)] for _ in range(numofarray)]

        for i in range(numofarray):
            for j in range(sizeofarray):
                matrix[i][j] = msg.data[(i * sizeofarray) + j]

        for i in range(sizeofarray):
            if (matrix[1][i] == 1):
                self.secondFloorCount += 1
            if (matrix[2][i] == 1):
                self.thirdFloorCount += 1

        # print("secondfloor :" + str(self.secondFloorCount))
        # print("thirdfloor :" + str(self.thirdFloorCount))

class LiftCurrentFloorSubscriber():
    def __init__(self):
        self.lift_current_floor_sub = rospy.Subscriber("wstation/lift_current_floor", Int8, callback=self._callback)
        self.lift_current_floor_buf = None

    def function(self):
        return self.lift_current_floor_buf
    
    def _callback(self, msg):
        self.lift_current_floor_buf = msg.data

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
        self.lift_item_status_sub = rospy.Subscriber("wstation/lift_item_status", Bool, callback=self._callback)
        self.lift_item_status_buf = None

    def function(self):
        return self.lift_item_status_buf
    
    def _callback(self, msg):
        self.lift_item_status_buf = msg.data

class LiftItemSizeSubscriber():
    def __init__(self):
        self.lift_item_size_sub = rospy.Subscriber("/wstation/lift_item_size", String, callback=self._callback)
        self.lift_item_size_buf = String()

    def function(self):        
        return self.lift_item_size_buf
    
    def _callback(self, msg):
        self.lift_item_size_buf = msg.data

class LifDestinationFloorSubscriber():
    def __init__(self):
        self.lift_destination_floor_sub = rospy.Subscriber("wstation/lift_destination_floor", Int8, callback=self._callback)
        self.lift_destination_floor_buf = None

    def function(self):        
        return self.lift_destination_floor_buf
    
    def _callback(self, msg):
        self.lift_destination_floor_buf = msg.data

class OcrStatucSubscriber():
    def __init__(self):
        self.ocr_status_sub = rospy.Subscriber("/wstation/ocr_status", String, callback=self._callback, queue_size=1)
        self.ocr_status_buf = Bool()

    def function(self):
        return self.ocr_status_buf

    def _callback(self, msg):
        self.ocr_status_buf = msg.data

class JamesReadySubscriber():
    def __init__(self):
        self.jamesarrived_sub = rospy.Subscriber("/wstation/james_ready", Bool, callback=self._callback)
        self.james_ready_flag = Bool()

    def function(self):        
        return self.james_ready_flag
    
    def _callback(self, msg):
        self.james_ready_flag = msg.data

class EmergencySubscriber():
    def __init__(self):
        self.emergency_sub = rospy.Subscriber("wstation/emergency", Bool, callback=self._callback)
        self.emergency_buf = Bool()

    def function(self):        
        return self.emergency_buf
    
    def _callback(self, msg):
        self.emergency_buf = msg.data

class ManualSubscriber():
    def __init__(self):
        self.manual_sub = rospy.Subscriber("wstation/manual", Bool, callback=self._callback)
        self.manual_buf = Bool()

    def function(self):        
        return self.manual_buf
    
    def _callback(self, msg):
        self.manual_buf = msg.data

# Publisher 

class StartItemSizeEstimationPublisher():
    # If item is inserted this camera trigger topic will sent
    def __init__(self):
        self.start_item_size_estimation_pub = rospy.Publisher("wstation/start_item_size_estimation", Bool, queue_size=1)
        self.msg = Bool()

    def send_flag(self, flag):
        self.msg.data = flag
        self.start_item_size_estimation_pub.publish(self.msg)

class StartOcrEstimationPublisher():
    def __init__(self):
        self.start_ocr_estimation_pub = rospy.Publisher("/wstation/start_ocr_estimation", Bool, queue_size=1)
        self.msg = Bool()

    def send_flag(self, flag):
        self.msg.data = flag
        self.start_ocr_estimation_pub.publish(self.msg)

class StationReadyPublisher():
    def __init__(self):
        self.station_ready_pub = rospy.Publisher("/wstation/station_ready", Bool, queue_size=1)
        self.msg = Bool()

    def send_flag(self, flag):
        self.msg.data = flag
        self.station_ready_pub.publish(self.msg)

class WstationStatusPublisher():
    def __init__(self):
        self.send_to_destination_pub = rospy.Publisher("wstation/status", String, queue_size=1)
        self.msg = String()

    def send_wstation_status(self,status):
        print("self.status :" + status)
        self.msg.data = status
        self.send_to_destination_pub.publish(self.msg)


######################################################################################################################
######################################################################################################################

class TopicList:

    flooritemstatus = -1           # array, first 4 is 1st floor, and so on
    liftcurrentfloor = 0           # 1, 2, 3
    liftstatus = "arrived"         # arrived, up, down
    liftitemstatus = False         # True, False
    liftitemsize = "none"          # good, bad, none
    ocrstatus = False              # True, False
    jamesready = False              # True, False 
    
    status = "none"                # Wstation Status
    emergency = False              # True, False
    manual = False                 # True, False


class WstationTask(TopicList):
    def __init__(self):
        print("Start Define Wsation Status")
    
    def definestatus(self):

        if self.emergency != True:

            if self.liftitemstatus == True:

                if (self.liftitemsize == "good") and (self.ocrstatus == "good"):

                    if self.liftcurrentfloor == 1:

                        if self.liftstatus == "arrived":

                            if self.jamesready == True:

                                self.status = "tojames"

                            else: # up, down

                                self.status = "waitjames"

                    else:
                        self.status = "gofirstfloor"

                elif self.liftitemsize == "bad":                
                    # self.ocrstatus == any status
                    if self.liftcurrentfloor == 1:

                        if self.liftstatus == "arrived":

                            self.status = "totray"

                        else: # up,down

                            self.status = "gofirstfloor"

                    else:
                        self.status = "gofirstfloor"

                else: # (self.liftitemsize == "none") or (self.ocrstatus == "none")

                    self.status = "gofirstfloor"

            elif self.liftitemstatus == False:

                if self.liftcurrentfloor == 1:

                    if self.flooritemstatus == -1:

                        self.status = "waititem"
                    
                    elif self.flooritemstatus == 2:

                        self.status = "gosecondfloor"
                    
                    elif self.flooritemstatus == 3:

                        self.status = "gothirdfloor"
                
                elif self.liftcurrentfloor == 2:

                    if (self.flooritemstatus == -1) and (self.liftdestinationfloor == -1):

                        self.status = "gofirstfloor"
                    
                    elif (self.flooritemstatus == 2) and (self.liftstatus == "arrived") and (self.liftdestinationfloor == 2):

                        self.status = "pushitem"
                    
                    elif (self.flooritemstatus == 3) and (self.liftstatus =="arrived") and (self.liftdestinationfloor == 2):

                        self.status = "gofirstfloor"
                    
                    elif (self.flooritemstatus == 3) and (self.liftstatus =="up") and (self.liftdestinationfloor == 3):

                        self.status = "gothirdfloor"
                    
                elif self.liftcurrentfloor == 3:
                    
                    if (self.flooritemstatus == -1) and (self.liftdestinationfloor == -1):

                        self.status = "gofirstfloor"
                    
                    elif (self.flooritemstatus == 2) and (self.liftstatus == "arrived") and (self.liftdestinationfloor == 3):

                        self.status = "gofirstfloor"
                    
                    elif (self.flooritemstatus == 3) and (self.liftstatus =="arrived") and (self.liftdestinationfloor == 3):
                    
                        self.status = "pushitem"
                    
                    elif (self.flooritemstatus == 2) and (self.liftstatus =="down") and (self.liftdestinationfloor == 3):

                        self.status = "gosecondfloor"

            else: # no data recieved

                self.status = "waitstatus"
                        
        elif self.emergency == True:

            if self.manual == True:

                self.status = "manual"
            
            elif self.manual == False:

                self.status = "emergency"

        
        return self.status

####################################################################################################################
####################################################################################################################

def wstation_main():
    rospy.init_node('Wstation_sub',anonymous=False)
    rate = rospy.Rate(10)

    floor_item_status_sub       = FloorItemStatusSubscriber()             # IrSensor list
    lift_current_floor_sub      = LiftCurrentFloorSubscriber()
    lift_status_sub             = LiftStatusSubscriber()
    lift_item_status_sub        = LiftItemStatusSubscriber()
    lift_item_size_sub          = LiftItemSizeSubscriber()
    lift_destination_floor_sub  = LifDestinationFloorSubscriber()

    james_ready_sub             = JamesReadySubscriber()                
    ocr_status_sub              = OcrStatucSubscriber()

    emergency_sub               = EmergencySubscriber()
    manual_sub                  = ManualSubscriber()

    wstation_task               = WstationTask()
    wstation_status             = WstationStatusPublisher()
    start_item_size_estimation_pub  = StartItemSizeEstimationPublisher()
    start_ocr_estimation_pub    = StartOcrEstimationPublisher()
    station_ready_pub           = StationReadyPublisher()

    initialize_pub = rospy.Publisher("/initialize", Bool, queue_size=1)

    defined_status = None
    is_ocr_trigger_off = False

    print("Wstation start")

    while not rospy.is_shutdown():

        rate.sleep()

        # Subscribe
        TopicList.flooritemstatus       = floor_item_status_sub.function()          # -1, 1, 2, 3
        TopicList.liftcurrentfloor      = lift_current_floor_sub.function()         # 1, 2, 3
        TopicList.liftstatus            = lift_status_sub.function()                # "wait", "move", "arrived"
        TopicList.liftitemstatus        = lift_item_status_sub.function()           # true, false
        TopicList.liftdestinationfloor  = lift_destination_floor_sub.function()     # 1, 2, 3
        TopicList.jamesready            = james_ready_sub.function()                # True, False

        # 제임스가 /wstation/station_ready 토픽을 받으면 
        if (TopicList.jamesready == True):
            print("initialize True!")
            initialize_pub.publish(True)
        else:
            initialize_pub.publish(False)        
        
        TopicList.emergency             = emergency_sub.function()                  # True, False
        TopicList.manual                = manual_sub.function()                     # True, False

        # status 가 "tojames" 나 "totray" 일 시 아래의 토픽을 받게 되면 전달되고 있는 도중 사이즈 측정 변동으로 인해 상태가 변해버릴 수 있음.
        if (defined_status != "tojames") and (defined_status != "totray"):
            TopicList.liftitemsize      = lift_item_size_sub.function()             # "good", "bad", "none"
            TopicList.ocrstatus         = ocr_status_sub.function()                 # "good", "bad", "none"

            if (TopicList.ocrstatus == "trigger_off"):
                is_ocr_trigger_off = True

            if (TopicList.liftitemstatus == False):
                is_ocr_trigger_off = False

        # Update station-ROS's status & Send status to action
        defined_status = wstation_task.definestatus()
        wstation_status.send_wstation_status(defined_status)

        # Send start item size & ocr estimation trigger!!
        if (TopicList.liftcurrentfloor == 1) and (TopicList.liftstatus == "arrived") and (TopicList.liftitemstatus == True):
            start_item_size_estimation_pub.send_flag(True)

            if (is_ocr_trigger_off == True):
                start_ocr_estimation_pub.send_flag(False)
            else:
                start_ocr_estimation_pub.send_flag(True)
                
        else:
            start_item_size_estimation_pub.send_flag(False)
            station_ready_pub.send_flag(False)

        # Send "wstation/station_ready" topic to "James"
        if (TopicList.liftitemsize == "good") and (TopicList.ocrstatus == "good"):
            station_ready_pub.send_flag(True)


if __name__ == '__main__':
    try:
        wstation_main()
    except rospy.ROSInterruptException:
        pass