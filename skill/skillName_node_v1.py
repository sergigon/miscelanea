#!/usr/bin/env python
# -*- coding: utf-8 -*-


__author__ = "Sergio Gonzalez Diaz"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Sergio Gonzalez Diaz"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Sergio Gonzalez Diaz"
__email__ = "sergigon@ing.uc3m.es"
__status__ = "Development"

# Skill libraries
from skill.skill import Skill, ActionlibException, CONDITIONAL
import rospkg
import rospy
import roslib
import importlib
import actionlib

# Messages
import skillName_skill.msg
from std_msgs.msg import String, Empty
from interaction_msgs.msg import CA
from hri_manager.key_value_pairs import to_dict

# Local libraries
from skillName.ca_functions import *
from skillName.exceptions_lib import PauseException, ErrorException

# Skill variables
# Package name
pkg_name = 'skillName_skill'
# Skill name (declare this only if the name is different of 'pkg_name')
skill_name = "skillName_skill"


class SkillNameSkill(Skill):
    """
    Weather skill class.
    """

    # Feedback and result of this skill
    _feedback = skillName_skill.msg.WeatherFeedback()
    _result = skillName_skill.msg.WeatherResult()
    _emitter = 'skillName_ca'

    # Constants

    # Params constants


    def __init__(self):
        """
        Init method.
        """

        # Class variables
        self._as = None # SimpleActionServer variable
        self._answer_received = '' # CA answer received

        # Goal control variables
        self._goal_exec = False # Indicates if goal is being handled
        self._pause_requested = False # Indicates if pause is requested
        self._pause = False # Indicates if pause
        self._step = '' # Indicates new process step
        self._exec_out = True # Indicates goal exec loop exit
        
        # Goal varaibles
        self._max_time,  self._time_run = 0, 0 # Max time to handle the goal; time running handling goal
        self._t0, self._t1 = 0, 0 # Variables to update self._time_run
        self._number_plays, self._i_plays = 0, 0 # Max number of plays to handle the goal; plays handling goal
        self._limit_method = '' # Indicates limit method selected (max time, plays or both)
        self._time_question = 10000 # Time to make a proactivity question

        # Local paths
        rospack = rospkg.RosPack()
        self._pkg_path = rospack.get_path(pkg_name) + '/' # Package path
        self._data_path = self._pkg_path + 'data/' # Data path

        # Tablet paths
        #self._icons_path = 'image/weather/' # Icons path

        # SkillName variables
        #self.location, self.forecast_type, self.date = '', '', ''

        # init the skill
        Skill.__init__(self, skill_name, CONDITIONAL)

######################### Skill callbacks #########################
    def response_callback(self, recog_response):
        """
        Receive the response from hri manager.
        Translate the response into a server param
        """

        if(recog_response.emitter == self._emitter):
            values = to_dict(recog_response.values)
            ans = values['answer_value'].lower()                   # turn it into lower case

            rospy.logdebug("Received response: " + ans)

            if ans in ["communicationProblems", "no_response"]:				# If there's some problem or user not response
                self._answer_received = 'Error'
            else:
                self._answer_received = ans

        else:
            rospy.logwarn('Response not expected')

    def create_msg_srv(self):
        """
        This function has to be implemented in the children.

        @raise rospy.ROSException: if the service is inactive.
        """
        rospy.loginfo("Start requested")

        # publishers and subscribers
        self.ca_pub = rospy.Publisher(
            "hri_manager/ca_activations", CA, queue_size=1) # CA publisher
        self.ca_deactivation_pub = rospy.Publisher(
            "hri_manager/ca_deactivations", String, queue_size=1) # CA deactivation publisher

        self.sub_response = rospy.Subscriber(
        	"hri_manager/response", CA, self.response_callback) # CA response

        #self.sub_response = rospy.Subscriber(robot + "hri_manager/response", CA, self.response_callback) # CA subscriber

        # servers and clients
        # Si el servidor actionlib no se ha inicializado:

        if not self._as:
            self._as = actionlib.SimpleActionServer(skill_name, weather_skill.msg.WeatherAction, execute_cb=self.execute_cb, auto_start=False)
            # start the action server
            self._as.start()

    def shutdown_msg_srv(self):
        """
        This function has to be implemented in the children.
        """

        # publishers and subscribers
        # FIXME: do not unregister publishers because a bug in ROS
        # self.__test_pub.unregister()

        rospy.loginfo("Stop requested")
        # servers and clients
        self.sub_response.unregister()

        # Cancel goal
        self._as.preempt_request = True

    def pause_exec(self):
        """
        Modify the variable self._pause if goal is being handled, when a pause is requested.
        """

        if(self._goal_exec): # Goal being handled
            self._pause_requested = True
            rospy.logdebug('Pause requested')
        else: # Goal NOT being handled
            rospy.logwarn('Goal not being handled')

    def resume_exec(self):
        """
        Modify the variable self._pause, when a resume is requested.
        """

        self._pause = False
        self._feedback.app_status = 'resume_ok'
        self._as.publish_feedback(self._feedback)
#=================================================================#

    def exception_check(self, deactivation = [], t0 = -1, t1 = -1):
        """
        Checks if an exception has been asked. It can be by a preempt request or by a pause request.

        @param deactivation: List of CA names to be deactivated (if exception requested)
        @param t0: Time 0 to use for time_run (if exception requested)
        @param t1: Time 1 to use for time_run (if exception requested)
        """

        # Parameters update
        if(self._pause_requested or self._as.is_preempt_requested()):
            # CA deactivation
            if(len(deactivation)>0):
                for msg_name in deactivation:
                    rospy.logdebug('Deactivating CA: %s' % msg_name)
                    self.ca_deactivation_pub.publish(msg_name)
            # Time update
            if(t0!=-1 and t1!=-1):
                self._time_run += t1 - t0

        # Raise exceptions
        ############# State Preempted checking #############
        # If goal is in Preempted state (that is, there    #
        # is a goal in the queue or the actual goal is     #
        # cancelled), the exception is activated.          #
        ####################################################
        if(self._as.is_preempt_requested()):
            self._pause_requested = False
            rospy.logwarn("Preempt requested")
            raise ActionlibException
        
        ###################### Pause #######################
        if(self._pause_requested):
            self._pause_requested = False
            self._pause = True
            rospy.logwarn('Raising Pause Exception')
            raise PauseException

    def pause_wait(self):
        """
        Pause loop. Finishes when goal is resumed or cancelled.
        """

        if(self._pause and not self._as.is_preempt_requested()):
	        rospy.loginfo('Start waiting')
	        while(self._pause and not self._as.is_preempt_requested()):
	            rospy.logdebug('waiting...')
	            rospy.sleep(1)

    
    def _goal_handler(self, goal):
        """
        Goal handler.

        Checks if the goal is appropriate. If true, it configures the variables for the goal execution.

        @param goal: Goal received.

        @return goal_accepted: True if goal is accepted, else, False.
        """

        # Fill goal variables
        # -- Skill command -- #
        skill_command_vec = goal.skill_command.split('/') # Divides goal by fields
        if(len(skill_command_vec)<3):
            rospy.logerr('NOT enough fields')
            return False
        #self.location = skill_command_vec[0] # Location ('madrid', or 'madrid, es')
        # ...
        # -- Max time -- #
        self._max_time = goal.max_time
        # -- Number plays -- #
        self._number_plays = goal.number_plays
        
        # Check proactivity
        if(goal.proactivity == 0):
            self._time_question = 50 # Get time question
        elif(goal.proactivity == 1):
            self._time_question = 40
        elif(goal.proactivity == 2):
            self._time_question = 30
        elif(goal.proactivity == 3):
            self._time_question = 20
        elif(goal.proactivity == 4):
            self._time_question = 10
        else:
            rospy.logerr('Proactivity level not specified')
            return False

        # Check goal variables
        # ...
        

        # Check max_time and number_plays
        if(self._max_time>0 and self._number_plays>0):
            self._limit_method = 'both'
        elif(self._max_time>0):
            self._limit_method = 'time'
        elif(self._number_plays>0):
            self._limit_method = 'plays'
        else:
            rospy.logerr('max_time and number_plays NOT accepted. Specify one of them')
            return False

        # Goal accepted
        rospy.loginfo('Goal accepted')
        return True

    def update_percentage(self):
        percentage_plays, percentage_time = 0, 0

        if(self._limit_method == 'both' or self._limit_method == 'plays'):
            percentage_plays = int((float(self._i_plays)/float(self._number_plays))*100)
            rospy.logdebug('percentage plays: %s' % percentage_plays)
        if(self._limit_method == 'both' or self._limit_method == 'time'):
            percentage_time = int((float(self._time_run)/float(self._max_time))*100)
            rospy.logdebug('percentage time: %s' % percentage_time)

        percentage = percentage_plays if percentage_plays > percentage_time else percentage_time
        percentage = percentage if percentage<100 else 100
        rospy.loginfo('percentage: %s' % percentage)
        self._feedback.percentage_completed = percentage

    def execute_cb(self, goal):
        """
        Callback of the node. Activated when a goal is received

        @param goal: weather_skill goal.
        """

        # Goal control variables
        self._pause_requested = False
        self._pause = False
        self._step = 'Process_goal'
        self._exec_out = False

        # Goal variables
        self._time_run, self._i_plays = 0, 0
        self._t0, self._t1 = 0, 0 # Variables to update self._time_run
        self._limit_method = ''
        n_questions = 1 # Variable to indicate the number of continue questions

        # Result default values
        self._result.skill_result = self._result.SUCCESS # Success

        # Feedback default values
        self._feedback.app_status = 'start_ok'
        self._feedback.percentage_completed = 0
        self._feedback.engagement = True
        # Publish initial feedback
        self._as.publish_feedback(self._feedback)

        ####################### Skill active #######################
        if self._status == self.RUNNING:
            self._goal_exec = True # Goal execution starts
            print('\n')
            rospy.loginfo("RUNNING...")
            ###################### Exec loop #######################
            while(not self._exec_out):
                try:
                    # Wait loop
                    self.pause_wait() # If wait is asked it enters a loop
                    self._feedback.app_status = ''
                    self._answer_received = ''

                    self.exception_check() # Checks if a exception is requested

                    # Process_goal
                    if(self._step=='Process_goal'):
                        rospy.loginfo('Goal: %s' % goal)
                        if(not self._goal_handler(goal)): # Goal NOT correct
                            raise ErrorException('Goal NOT correct')
                        self._step = 'Get_weather'
                    self.exception_check() # Checks if a exception is requested

                    t0 = time.time()

                    # Get weather standard data
                    if(self._step=='Get_weather'):
                    	rospy.loginfo("Making weather request")
                        weather_dic = Weather(self._data_path)._request_weather(self.location, self.forecast_type, self.date) # Get the weather info
                        if(weather_dic == -1):
                            raise ErrorException('Error getting weather')
                        self._step = 'Display'
                    self.exception_check(t0 = t0, t1 = time.time())

                    # Display weather
                    if(self._step=='Display'):
                    	rospy.loginfo("Displaying info") 
                        if(self._manage_display(self.forecast_type, self.date, weather_dic)):
                             raise ErrorException('Error managing display')

                        # Update limit variables
                        self._time_run += time.time() - t0
                        self._i_plays += 1
                        self.update_percentage()
                        self._step = 'Get_weather'

                    # Exit while
                    self._step = 'Out' if self._feedback.percentage_completed>=100 else self._step # Exit if limits are exceeded (number_plays, max_time)
                    
                    # Asks when a time has passed
                    if(self._time_run > self._time_question * n_questions):
                        try:
                            language = rospy.get_param(self._LANG_PARAM) # Get langauge
                        except:
                            language = 'es'
                            rospy.logwarn("Language not found. Using language %s" % language)
                        ca_info = makeCA_ASR_question(etts_text='¿Quieres continuar?', language=str(language), grammar = "si_no.gram", answer_id = 'sino', emitter=self._emitter)
                        self.ca_pub.publish(ca_info)

                        values = to_dict(ca_info.values)
                        t_ans_0 = time.time()
                        t_ans_1 = time.time()
                        while(self._answer_received == '' and t_ans_1-t_ans_0 < (int(values['answer_time'])+3) * int(values['answer_attempts'])):
                            rospy.logdebug('Waiting response...')
                            rospy.sleep(1)
                            t_ans_1 = time.time()
                        if(self._answer_received == 'si'): # Continue
                            self._feedback.engagement = True
                        elif(self._answer_received == 'no'): # Stops skill
                            raise ActionlibException
                            self._step = 'Out'
                        else: # Continue but changes engagement
                            self._feedback.engagement = False
                        n_questions += 1

                    if(self._step == 'Out'):
                        # Exit while
                        self._exec_out = True
                    else:
                        self._exec_out = False

                #################### Exceptions ####################
                ### Preempted or cancel:
                except ActionlibException:
                    rospy.logwarn('Preempted or cancelled')
                    self._exec_out = True
                    if(self._status == self.STOPPED):
                        self._feedback.app_status = 'stop_ok'
                    else:
                        self._feedback.app_status = 'cancel_ok'
                    self._result.skill_result = self._result.FAIL # Preempted
                ### Error
                except ErrorException as e:
                    rospy.logerr(e)
                    self._exec_out = True
                    self._result.skill_result = self._result.ERROR # Error
                ### Pause
                except PauseException:
                    rospy.logwarn('Paused')
                    rospy.loginfo('Next step: %s' % self._step)
                    self._feedback.app_status = 'pause_ok'
                    self._exec_out = False
                #=================== Exceptions ===================#

                # Publish feedback at the end of loop
                self._as.publish_feedback(self._feedback)

            #===================== Exec loop ======================#
            self._goal_exec = False # Goal execution finished
            print('\n')
        #==================== Skill active ========================#

        ##################### Skill NOT active #####################
        else:
            rospy.logwarn("STOPPED")
            rospy.logwarn("[%s] Cannot send a goal when the skill is stopped" % pkg_name)
            self._result.skill_result = self._result.FAIL # Error
        #==========================================================#
        
        ### Result sending (with feedback) and goal status update ##
        if self._result.skill_result == self._result.SUCCESS:
            rospy.logdebug("setting goal to succeeded")
            self._feedback.app_status = 'completed_ok'
            self._as.publish_feedback(self._feedback)
            self._as.set_succeeded(self._result)
        else:
            rospy.logdebug("setting goal to preempted")
            self._feedback.app_status = 'completed_fail'
            self._as.publish_feedback(self._feedback)
            self._as.set_preempted(self._result)
        rospy.loginfo("#############################")
        rospy.loginfo("######## Result sent ########")
        rospy.loginfo("#############################")
        #==========================================================#



if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(skill_name, log_level=rospy.DEBUG)
        rospy.loginfo('[' + pkg_name + ': ' + skill_name + ']')

        # create and spin the node
        node = SkillNameSkill()
        rospy.sleep(1)
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass