#!/usr/bin/env python

import rospy
import time

from decenter_msgs.msg import *

from robotnik_fms_msgs.srv import *

from rcomponent.rcomponent import *

from std_srvs.srv import *

from robotnik_signal_msgs.msg import *

from robotnik_signal_msgs.srv import *

from robotnik_msgs.msg import *

from robotnik_msgs.srv import *

class DecenterFMSManager(RComponent):
    def __init__(self):

        RComponent.__init__(self)

        self.last_mission = []
        self.led_ev = False
        self.checking_robot_in_mission = False
        self.robot_in_mission = False
        self.robot_id_enable_node = -1
        self.node_enable_wait_time = 15
        #self.img_enable_wait_time = 300
        self.robot_enable_service = []
        self.send_pictures_enabled = True

    def rosReadParams(self):

        """Gets params from param server"""
        RComponent.rosReadParams(self)

        self.img_enable_wait_time = rospy.get_param(
            'img_enable_wait_time',
            default="301"
        )

        rospy.loginfo(
            "Image cooldown time: " +
            str(self.img_enable_wait_time)
        )
        self.nodes_selected_param = rospy.get_param(
            'nodes_selected',
            default="101 102"
        )
        self.lights_service_param = rospy.get_param(
            'lights_service_name',
            default="/gazebo/set_light_properties"
        )

        self.node_selected = []
        rospy.loginfo(
            "Nodes param: " +
            self.nodes_selected_param
        )

        for node_str in self.nodes_selected_param.split():
            node = int(node_str)
            self.node_selected.append(node)
            rospy.loginfo(
                "Node to block: " +
                node_str
            )
        print(self.node_selected)
        robot_param = rospy.get_name()
        robot_param += "/robots"
        self.robots = rospy.get_param(
            robot_param
        )
        # rospy.loginfo(
        #     self.robots[0]['id']
        # )
        #filter(lambda obj: obj.get('id') == 0, self.robots)

    def rosSetup(self):

        """Creates and inits ROS components"""
        RComponent.rosSetup(self)
        rospy.Timer(
            period=rospy.Duration(1),
            callback=self.mission_check_timer_callback,
            oneshot=False
        )
        self.topic_sub = rospy.Subscriber(
            '/decenter_fms_manager/object_detector_mqtt_msg',
            ObjectDetector,
            self.object_detector_cb,
            queue_size=10
        )

        return 0

    def initState(self):

        """ Actions perfomed in init state"""
        self.switchToState(State.READY_STATE)

    def led_event_timer_callback(self, event):
        # rospy.loginfo('led event timer callback')
        self.clear_light_indicator()
        self.led_ev = False
        self.robot_in_mission = False

    def mission_check_timer_callback(self, event):
        if self.led_ev:
            return True
        if self.checking_robot_in_mission:
            return False
        self.checking_robot_in_mission = True
        if self.is_robot_in_mission():
            if not self.robot_in_mission:
                self.send_normal_route_indicator()
                rospy.loginfo('robot in mission')
            self.robot_in_mission = True
        else:
            if self.robot_in_mission:
                self.clear_light_indicator()
                rospy.loginfo('robot idle')
            self.robot_in_mission = False
        self.checking_robot_in_mission = False

    def node_enable_timer_callback(self, event):
        # rospy.loginfo('node enable timer callback')
        if self.enable_node(self.node_selected):
            rospy.loginfo('node %s enabled', self.node_selected)

    def enable_image_timer_callback(self, event):
        rospy.loginfo('Enable image timer callback')
        self.enable_send_pictures(
            enable=True,
            service=self.robot_enable_service
        )


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

    def enable_send_pictures(self, enable, service):
        rospy.loginfo('Waiting for service')
        self.send_pictures_enabled = enable
        try:
            rospy.wait_for_service(service, 10.0)
        except rospy.ServiceException as e:
            rospy.loginfo("Service wait timeout: %s" % e)
            return False

        try:
            self._enable_service = rospy.ServiceProxy(
                name=service,
                service_class=SetBool,
            )
            response = self._enable_service(enable)
            rospy.loginfo(response)
            return response.success
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s" % e)
            return False

    def object_detector_fail(self, service):
        rospy.logerr('Something went wrong')
        rospy.Timer(
            period=rospy.Duration(
                self.img_enable_wait_time
            ),
            callback=self.enable_image_timer_callback,
            oneshot=True
        )
        # rospy.sleep(30)
        # self.enable_send_pictures(
        #     enable=True,
        #     service=service
        # )
        return

    def object_detector_cb(self, msg):
        '''
            Callback object_detector_cb (rostopic pub /object_detector_mqtt_msg decenter_msgs/ObjectDetector )
        '''
        # rospy.logdebug(
        rospy.loginfo(
            'object_detector_cb received:: %s' % msg
        )

        #if there are not objects return
        if len(msg.objects) <= 0:
            rospy.logwarn(
                'object_detector_cb::objects is empty'
            )
            return

        robot_data = filter(
            lambda obj: obj.get('id') == int(msg.metadata.robot_id),
            self.robots
        )
        if not self.send_pictures_enabled:
            rospy.logwarn(
                'Sending picture is not enabled, doing nothing'
            )
            return
        robot_enable_service = robot_data[0]['enable_service']
        self.robot_enable_service = robot_enable_service
        self.enable_send_pictures(
            enable=False,
            service=robot_enable_service
        )

        #so far we just take the first object detected
        object_type = msg.objects[0].warehouse_obj_class
        rospy.logdebug(
            'Identified the object: %s'%object_type
        )

        #switch between 3 cases: 'other', 'robot', 'person'
        if object_type == 'person':

            rospy.loginfo(
                'Person Detected !'
            )

            self.send_person_alert(int(msg.metadata.robot_id))



        elif object_type == 'robot':

            rospy.loginfo(
                'Robot Detected !'
            )


            if not self.send_robot_indicator(int(msg.metadata.robot_id)):
                self.object_detector_fail(robot_enable_service)
                return

            #so far the node to disable is hardcoded but parametrized
            if not self.get_current_mission(int(msg.metadata.robot_id)):
                self.object_detector_fail(robot_enable_service)
                return
            if not self.cancel_mission(int(msg.metadata.robot_id)):
                self.object_detector_fail(robot_enable_service)
                return
            rospy.sleep(5)
            if not self.enable_node(self.node_selected):
                self.object_detector_fail(robot_enable_service)
                return
            if not self.unblock_node(
                    node=self.node_selected,
                    robot_id=int(msg.metadata.robot_id)
            ):
                self.object_detector_fail(robot_enable_service)
                return

            if not self.disable_node(self.node_selected):
                self.object_detector_fail(robot_enable_service)
                return
            if not self.insert_last_mission():
                self.object_detector_fail(robot_enable_service)
                return
            if not self.wait_until_robot_takes_new_mission(
                    int(msg.metadata.robot_id)
            ):
                self.object_detector_fail(robot_enable_service)
                return

            rospy.Timer(
                period=rospy.Duration(
                    self.node_enable_wait_time
                ),
                callback=self.node_enable_timer_callback,
                oneshot=True
            )

        elif object_type == 'others':

            rospy.loginfo('Other thing Detected')
            if not self.send_others_indicator(int(msg.metadata.robot_id)):
                self.object_detector_fail(robot_enable_service)
                return
            #so far the node to disable is hardcoded but parametrized
            if not self.get_current_mission(int(msg.metadata.robot_id)):
                self.object_detector_fail(robot_enable_service)
                return
            if not self.cancel_mission(int(msg.metadata.robot_id)):
                self.object_detector_fail(robot_enable_service)
                return
            rospy.sleep(5)
            if not self.enable_node(self.node_selected):
                self.object_detector_fail(robot_enable_service)
                return
            if not self.unblock_node(
                    node=self.node_selected,
                    robot_id=int(msg.metadata.robot_id)
            ):
                self.object_detector_fail(robot_enable_service)
                return

            if not self.disable_node(self.node_selected):
                self.object_detector_fail(robot_enable_service)
                return
            if not self.insert_last_mission():
                self.object_detector_fail(robot_enable_service)
                return
            if not self.wait_until_robot_takes_new_mission(
                    int(msg.metadata.robot_id)
            ):
                self.object_detector_fail(robot_enable_service)
                return
        else:
            rospy.logwarn(
                'Indeterminate case. Not person, not robot, not others'
            )
            self.enable_send_pictures(
                enable=True,
                service=robot_enable_service
            )
            return

        # Success
        rospy.loginfo('Succeed')
        rospy.Timer(
            period=rospy.Duration(
                self.img_enable_wait_time
            ),
            callback=self.enable_image_timer_callback,
            oneshot=True
        )
        # rospy.sleep(30)
        # self.enable_send_pictures(
        #     enable=True,
        #     service=robot_enable_service
        # )
        return

    def unblock_node(
            self,
            node,
            robot_id,
            retry=10
    ):
        for retry_count in range(0, retry):
            if self.unblock_node_core(
                    node=node,
                    robot_id=robot_id,
            ):
                return True
            rospy.sleep(0.25)
        return False

    def unblock_node_core(self, node, robot_id):

        rospy.loginfo(
            'Unblocking node:%s'%node
        )

        try:
            block_node = rospy.ServiceProxy(
                '/robotnik_fms_routes_node/block_node',
                BlockNode
            )
            block_node_srv_msg = BlockNodeRequest()
            block_node_srv_msg.node_id = node
            block_node_srv_msg.robot_id = robot_id
            block_node_srv_msg.block = False
            block_node_srv_msg.reserve = False
            response = block_node(block_node_srv_msg)

        except rospy.ServiceException as e:
            rospy.logerr(
                "block_node service call failed: %s"%e
            )
            return False

        rospy.logdebug(
            'Received response from service:%s'%response
        )
        return True


    def enable_node(self, node, retry=10):
        rospy.loginfo(
            'Enabling node:%s'%node
        )
        for _ in range(retry):
            if self.disable_node_core(node, False):
                return True
            rospy.sleep(0.25)
        rospy.logerr('No more retries')
        return False

    def disable_node(self, node, retry=10):
        rospy.loginfo(
            'Disabling node:%s'%node
        )
        for _ in range(retry):
            if self.disable_node_core(node):
                return True
            rospy.sleep(0.25)
        rospy.logerr('No more retries')
        return False

    def disable_node_core(
            self,
            node,
            disable=True
    ):
        try:
            disable_node = rospy.ServiceProxy(
                '/robotnik_fms_routes_node/disable_node',
                DisableNode
            )
            disable_node_srv_msg = DisableNodeRequest()
            disable_node_srv_msg.node_id = node
            disable_node_srv_msg.disable = disable
            response = disable_node(disable_node_srv_msg)

        except rospy.ServiceException as e:
            rospy.logerr(
                "disable_node service call failed: %s"%e
            )
            return False

        rospy.logdebug(
            'Received response from service:%s'%response
        )

        return True

    def get_current_mission(self, robot_id):

        rospy.loginfo(
            'Getting current mission of robot %s'%robot_id
        )

        try:
            get_mission = rospy.ServiceProxy(
                '/robotnik_fms_ddbb_manager/Missions/get',
                GetMissions
            )
            get_mission_srv_msg = GetMissionsRequest()
            get_mission_srv_msg.id = 0
            get_mission_srv_msg.robot_id = robot_id
            response = get_mission(get_mission_srv_msg)
        except rospy.ServiceException as e:
            rospy.logerr(
                "get_current_mission service call failed: %s"%e
            )
            return False

        rospy.logdebug(
            'Received response from service:%s'%response
        )

        #check robot has missions
        if len(response.missions) <= 0:
            rospy.logwarn(
                'Robot %s has no missions assigned'%robot_id
            )
            return False
        #save mission parameters
        self.last_mission = response.missions[0]

        return True

    def cancel_mission(self, robot_id):

        rospy.loginfo(
            'Canceling current mission of robot %s'%robot_id
        )

        try:
            cancel_mission = rospy.ServiceProxy(
                '/robotnik_fms_dispatcher/cancel_mission',
                RobotNodesTasks
            )
            cancel_mission_srv_msg = RobotNodesTasksRequest()
            cancel_mission_srv_msg.robot = robot_id
            response = cancel_mission(cancel_mission_srv_msg)
        except rospy.ServiceException as e:
            rospy.logerr(
                "cancel_mission service call failed: %s"%e
            )
            return False

        rospy.logdebug(
            'Received response from service:%s'%response
        )

        return True

    def insert_last_mission(self):

        rospy.loginfo('Inserting last mission again')

        if self.last_mission == [] :
            rospy.logerr(
                'Call to insert last mission with any last mission saved'
            )
            return False

        try:
            insert_mission = rospy.ServiceProxy(
                '/robotnik_fms_ddbb_manager/Missions/insert',
                InsertMission
            )
            insert_mission_srv_msg = InsertMissionRequest()
            insert_mission_srv_msg.mission = self.last_mission
            response = insert_mission(insert_mission_srv_msg)
        except rospy.ServiceException as e:
            rospy.logerr(
                "insert_last_mission service call failed: %s"%e
            )
            return False

        rospy.logdebug(
            'Received response from insert_mission_service:%s'%response
        )

        return True

    def clear_light_indicator(
            self,
            robot_prefix='robot_0/rb1_base',
            service_name='leds_driver/clear_signals',
    ):
        full_service_name = '/'
        full_service_name += robot_prefix
        full_service_name += '/'
        full_service_name += service_name
        try:
            send_light_alert = rospy.ServiceProxy(
                name=full_service_name,
                service_class=Trigger,
            )
            response = send_light_alert()
            if not response:
                rospy.logerr(
                    'The command returned error'
                )
        except rospy.ServiceException as e:
            raise e
            rospy.logerr(
                'Could not perform last command due to execption'
            )
            return False
        return True

    def manage_ligths(
            self,
            robot_prefix,
            service_name,
            signal,
            enable,
    ):

        full_service_name = '/'
        full_service_name += robot_prefix
        full_service_name += '/'
        full_service_name += service_name
        try:
            send_light_alert = rospy.ServiceProxy(
                name=full_service_name,
                service_class=SetSignal,
            )
            set_signal_srv_msg = SetSignalRequest()
            set_signal_srv_msg.signal_id = signal
            set_signal_srv_msg.enable = enable
            response = send_light_alert(set_signal_srv_msg)
            if not response:
                rospy.logerr(
                    'The command returned error'
                )
        except rospy.ServiceException as e:
            raise e
            rospy.logerr(
                'Could not perform last command due to execption'
            )
            return False
        return True


    def manage_buzzer_low_level(
            self,
            service_name,
            digital_ouput,
            enable,
    ):
        try:
            send_sound_alert = rospy.ServiceProxy(
                name=service_name,
                service_class=set_digital_output,
            )
            set_digital_output_srv_msg = set_digital_outputRequest()
            set_digital_output_srv_msg.output = digital_ouput
            set_digital_output_srv_msg.value = enable
            response = send_sound_alert(set_digital_output_srv_msg)
            if not response:
                rospy.logerr(
                    'The command returned error'
                )
        except rospy.ServiceException as service_expection:
            raise service_expection
            rospy.logerr(
                'Could not perform last command due to execption'
            )
            return False
        return True


    def manage_buzzer(
            self,
            robot_prefix,
            service_name,
            digital_ouput,
            period=1,
            on_time=0.5,
            iterations=10,
    ):
        full_service_name = '/'
        full_service_name += robot_prefix
        full_service_name += '/'
        full_service_name += service_name
        off_time = period - on_time
        if off_time < 0:
            rospy.logerr(
                'On time could not be greater and period'
            )
            return False
        if iterations <= 0:
            rospy.logerr(
                'Iterations should be greater than 0'
            )
            return False
        for _ in range(iterations):
            response = self.manage_buzzer_low_level(
                service_name=full_service_name,
                digital_ouput=digital_ouput,
                enable=True,
            )
            if not response:
                rospy.logerr('Error on buzzer on')
                return False
            rospy.sleep(on_time)
            response = self.manage_buzzer_low_level(
                service_name=full_service_name,
                digital_ouput=digital_ouput,
                enable=False,
            )
            if not response:
                rospy.logerr('Error on buzzer off')
                return False
            rospy.sleep(off_time)

        return True

    def send_normal_route_indicator(self, robot_id=0):
        # ROBOT HARDCODED
        # HARDWARE HARDCODED
        rospy.loginfo(
            'Sending Others normal_route indicator %s'%robot_id
        )

        rospy.loginfo('Enabling light signals')
        light_response = self.manage_ligths(
            robot_prefix='robot_0/rb1_base',
            service_name='leds_driver/set_signal',
            signal='reroute',
            enable=True,

        )
        return True

    def send_reroute_indicator(self, robot_id=0):
        # ROBOT HARDCODED
        # HARDWARE HARDCODED
        rospy.loginfo(
            'Sending Others reroute indicator %s'%robot_id
        )

        rospy.loginfo('Enabling light signals')
        light_response = self.manage_ligths(
            robot_prefix='robot_0/rb1_base',
            service_name='leds_driver/set_signal',
            signal='reroute',
            enable=True,

        )
        return True

    def send_others_indicator(self, robot_id=0):
        # ROBOT HARDCODED
        # HARDWARE HARDCODED
        self.led_ev = True
        rospy.Timer(
            period=rospy.Duration(5),
            callback=self.led_event_timer_callback,
            oneshot=True
        )
        # rospy.loginfo(
        #     'Sending Others light indicator %s'%robot_id
        # )

        # rospy.loginfo('Enabling light signals')
        light_response = self.manage_ligths(
            robot_prefix='robot_0/rb1_base',
            service_name='leds_driver/set_signal',
            signal='obstacle',
            enable=True,

        )
        if not light_response:
            rospy.logerr('Error while Enabling light signals')
            return False

        # rospy.loginfo('Enabling sound signals')
        sound_response = self.manage_buzzer(
            robot_prefix='robot_0/rb1_base',
            service_name='robotnik_base_hw/set_digital_output',
            digital_ouput=1,
            period=0.5,
            on_time=0.2,
            iterations=2,
        )
        if not sound_response:
            rospy.logerr('Error while Enabling sound signals')
            return False
        return True

    def send_robot_indicator(self, robot_id=0):
        # ROBOT HARDCODED
        # HARDWARE HARDCODED
        self.led_ev = True
        rospy.Timer(
            period=rospy.Duration(5),
            callback=self.led_event_timer_callback,
            oneshot=True
        )
        # rospy.loginfo(
        #     'Sending Robot light indicator %s'%robot_id
        # )

        # rospy.loginfo('Enabling light signals')
        light_response = self.manage_ligths(
            robot_prefix='robot_0/rb1_base',
            service_name='leds_driver/set_signal',
            signal='robot',
            enable=True,

        )
        if not light_response:
            rospy.logerr('Error while Enabling light signals')
            return False

        # rospy.loginfo('Enabling sound signals')
        sound_response = self.manage_buzzer(
            robot_prefix='robot_0/rb1_base',
            service_name='robotnik_base_hw/set_digital_output',
            digital_ouput=1,
            period=1.0,
            on_time=0.9,
            iterations=1,
        )
        if not sound_response:
            rospy.logerr('Error while Enabling sound signals')
            return False
        return True


    def send_person_alert(self, robot_id=0):
        # ROBOT HARDCODED
        # HARDWARE HARDCODED
        self.led_ev = True
        rospy.Timer(
            period=rospy.Duration(5),
            callback=self.led_event_timer_callback,
            oneshot=True
        )
        # rospy.loginfo(
        #     'Sending alert to Person robot %s'%robot_id
        # )

        # rospy.loginfo('Enabling light signals')
        light_response = self.manage_ligths(
            robot_prefix='robot_0/rb1_base',
            service_name='leds_driver/set_signal',
            signal='emergency',
            enable=True,

        )
        if not light_response:
            rospy.logerr('Error while Enabling light signals')
            return False

        # rospy.loginfo('Enabling sound signals')
        sound_response = self.manage_buzzer(
            robot_prefix='robot_0/rb1_base',
            service_name='robotnik_base_hw/set_digital_output',
            digital_ouput=1,
            period=0.5,
            on_time=0.25,
            iterations=4,
        )
        if not sound_response:
            rospy.logerr('Error while Enabling sound signals')
            return False

        # rospy.loginfo('Disabling light signals')
        # light_response = self.clear_light_indicator()
        # if not light_response:
        #     rospy.logerr('Error while disabling light signals')
        #     return False

        return True
        # #call gazebo robot service to blink the lights
        # try:
        #     send_person_alert = rospy.ServiceProxy(self.lights_service_param, SetBool)
        #     response = send_person_alert(True)
        # except rospy.ServiceException as e:
        #     rospy.logerr("send_person_alert service call failed: %s"%e)
        #     return False

        # rospy.logdebug('Received response from send_person_alert_service:%s'%response)

        # #wait some time
        # time.sleep(5)

        # #call gazebo robot service to switch off the lights
        # try:
        #     send_person_alert = rospy.ServiceProxy(self.lights_service_param, SetBool)
        #     response = send_person_alert(False)
        # except rospy.ServiceException as e:
        #     rospy.logerr("send_person_alert service call failed: %s"%e)
        #     return False

        # rospy.logdebug('Received response from send_person_alert_service:%s'%response)

        # rospy.sleep(60)
        # return True

    def wait_until_robot_takes_new_mission(
        self,
        robot_id=0,
        tries=10,
        sleep=1.0,
    ):
        get_mission = rospy.ServiceProxy(
            '/robotnik_fms_ddbb_manager/Missions/get',
            GetMissions
        )

        get_mission_srv_msg = GetMissionsRequest()
        get_mission_srv_msg.id = 0
        get_mission_srv_msg.robot_id = robot_id

        response = GetMissionsResponse()

        while(len(response.missions) <= 0 and tries > 0):

            try:
                response = get_mission(get_mission_srv_msg)
            except rospy.ServiceException as e:
                rospy.logerr(
                    "robot_takes_new_mission service call failed: %s"%e
                )
            rospy.logdebug(
                'Received response from service:%s'%response
            )

            time.sleep(sleep)
            tries = tries -1

        if tries <= 0:
            return False
        else:
            return True


    def is_robot_in_mission(
        self,
        robot_id=0,
        tries=10,
        sleep=1.0,
    ):
        get_mission = rospy.ServiceProxy(
            '/robotnik_fms_ddbb_manager/Missions/get',
            GetMissions
        )

        get_mission_srv_msg = GetMissionsRequest()
        get_mission_srv_msg.id = 0
        get_mission_srv_msg.robot_id = robot_id

        response = GetMissionsResponse()

        retrieved = False

        while not retrieved:

            try:
                response = get_mission(get_mission_srv_msg)
                if len(response.missions) > 0:
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                rospy.logerr(
                    "robot_takes_new_mission service call failed: %s"%e
                )
            rospy.logdebug(
                'Received response from service:%s'%response
            )
            time.sleep(sleep)
            tries = tries -1
            if tries == 0:
                rospy.loginfo('Error while trying to get if robot is in mission')
                return False