#!/usr/bin/env python

import rospy
import time

from decenter_msgs.msg import *

from robotnik_fms_msgs.srv import *

from rcomponent.rcomponent import *

class DecenterFMSManager(RComponent):
    def __init__(self):

        RComponent.__init__(self)

        self.last_mission = []

    def rosReadParams(self):

        """Gets params from param server"""
        RComponent.rosReadParams(self)

    def rosSetup(self):

        """Creates and inits ROS components"""
        RComponent.rosSetup(self)

        self.topic_sub = rospy.Subscriber('/decenter_fms_manager/object_detector_mqtt_msg', ObjectDetector, self.object_detector_cb, queue_size = 10)

        return 0

    def initState(self):

        """ Actions perfomed in init state"""
        self.switchToState(State.READY_STATE)

    def readyState(self):

        """Actions performed in ready state"""

    def shutdown(self):

        """Shutdowns device
        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)

    def switchToState(self, new_state):

        """Performs the change of state"""
        return RComponent.switchToState(self, new_state)

    def object_detector_cb(self, msg):
        '''
    		Callback object_detector_cb (rostopic pub /object_detector_mqtt_msg decenter_msgs/ObjectDetector )
    	'''
        rospy.logdebug('object_detector_cb received:: %s' % msg)

        #if there are not objects return
        if len(msg.objects) <= 0:
            rospy.logwarn('object_detector_cb::objects is empty')
            return

        #so far we just take the first object detected
        object_type = msg.objects[0].warehouse_obj_class
        rospy.logdebug('Identified the object: %s'%object_type)

        #switch between 3 cases: 'other', 'robot', 'person'
        if object_type == 'person':

            rospy.loginfo('Person Detected !')

            self.send_alert(msg.metadata.robot_id)

            return

        elif object_type == 'robot':

            rospy.loginfo('Robot Detected !')

            #so far the node to disable is hardcoded [node 6]
            self.disable_node(6)
            self.get_current_mission(msg.metadata.robot_id)
            self.cancel_mission(msg.metadata.robot_id)
            self.insert_last_mission()
            # TODO: do we need to wait here?
            time.sleep(0.5)
            self.enable_node(6)

            return

        elif object_type == 'others':

            if len(msg.objects[0].detection_details) <= 0 :
                rospy.loginfo('Other thing Detected')
            else:
                rospy.loginfo('Other thing Detected :: %s! '%msg.objects[0].detection_details.object_class)
            return

            #so far the node to disable is hardcoded [node 6]
            self.disable_node(6)
            self.get_current_mission(msg.metadata.robot_id)
            self.cancel_mission(msg.metadata.robot_id)
            self.insert_last_mission()

        else:
            rospy.logwarn('Indeterminate case. Not person, not robot, not others')
            return

    def disable_node(self, node):

        rospy.loginfo('Disabling node:%s'%node)

        try:
            disable_node = rospy.ServiceProxy('/robotnik_fms_routes_node/disable_node', DisableNode)
            disable_node_srv_msg = DisableNodeRequest()
            disable_node_srv_msg.node_id = node
            disable_node_srv_msg.disable = True
            response = disable_node(disable_node_srv_msg)

        except rospy.ServiceException as e:
            rospy.logerr("disable_node service call failed: %s"%e)
            return

        rospy.logdebug('Received response from service:%s'%response)

        return

    def get_current_mission(self, robot_id):

        rospy.loginfo('Getting current mission of robot %s'%robot_id)

        try:
            get_mission = rospy.ServiceProxy('/robotnik_fms_ddbb_manager/Missions/get', GetMissions)
            get_mission_srv_msg = GetMissionsRequest()
            get_mission_srv_msg.id = 0
            get_mission_srv_msg.robot_id = robot_id
            response = get_mission(get_mission_srv_msg)
        except rospy.ServiceException as e:
            rospy.logerr("get_current_mission service call failed: %s"%e)
            return

        rospy.logdebug('Received response from service:%s'%response)

        #check robot has missions
        if len(response.missions) <= 0:
            rospy.logwarn('Robot %s has no missions assigned'%robot_id)
            return
        #save mission parameters
        self.last_mission = response.missions[0]

        return

    def cancel_mission(self, robot_id):

        rospy.loginfo('Canceling current mission of robot %s'%robot_id)

        try:
            cancel_mission = rospy.ServiceProxy('/robotnik_fms_dispatcher/cancel_mission', RobotNodesTasks)
            cancel_mission_srv_msg = RobotNodesTasksRequest()
            cancel_mission_srv_msg.robot = robot_id
            response = cancel_mission(cancel_mission_srv_msg)
        except rospy.ServiceException as e:
            rospy.logerr("cancel_mission service call failed: %s"%e)
            return

        rospy.logdebug('Received response from service:%s'%response)

        return

    def insert_last_mission(self):

        rospy.loginfo('Inserting last mission again')

        if self.last_mission == [] :
            rospy.logerr('Call to insert last mission with any last mission saved')

        try:
            insert_mission = rospy.ServiceProxy('/robotnik_fms_ddbb_manager/Missions/insert', InsertMission)
            insert_mission_srv_msg = InsertMissionRequest()
            insert_mission_srv_msg.mission = self.last_mission
            response = insert_mission(insert_mission_srv_msg)
        except rospy.ServiceException as e:
            rospy.logerr("insert_last_mission service call failed: %s"%e)
            return

        rospy.logdebug('Received response from insert_mission_service:%s'%response)

        return

    def enable_node(self, node):

        rospy.loginfo('Enabling the node %s again'%node)



        return

    def send_alert(self, robot_id):

        rospy.loginfo('Sending alert to robot %s'%robot_id)

        #       TODO:call gazebo robot service to blink the lights
        #try:
        #    send_alert = rospy.ServiceProxy('TBD', TBD_SRVS_TYPE)
        #    send_alert_srv_msg = TBD_Request()
        #    send_alert_srv_msg.robot_id = robot_id
        #    response = send_alert(send_alert_srv_msg)
        #except rospy.ServiceException as e:
        #    rospy.logerr("send_alert service call failed: %s"%e)
        #    return
#
        #rospy.logdebug('Received response from send_alert_service:%s'%response)


        return



