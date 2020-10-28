#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String, Bool
import random
from geometry_msgs.msg import Point


# define state NORMAL
class Normal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['play','sleep', 'coords'],
                                    input_keys = ['norm_command', 'norm_flag'],
                                    output_keys = ['norm_coords'])
        self.normal_counter = 1

        self.pub = rospy.Publisher('move_coords', Point, queue_size=10)
        self.sub_command = rospy.Subscriber('command', String, cb_command)
        self.sub_flag = rospy.Subscriber('arrived', Bool, cb_flag)


    def execute(self, userdata):
        global sm_command, sm_flag

        self.normal_counter = 1
        time.sleep(1)
        rospy.loginfo('Executing state NORMAL')

        if sm_command == "play":
            print(sm_command)
            sm_command = None
            return 'play'
        
        else:
            normal_times = random.randrange(1,5)

            x = random.randrange(0,1000)
            y = random.randrange(0,1000)

            normal_coord = Point(x = x, y = y)

            print("Robot acting normal")
            print("Times: " + str(normal_times))
            print("Counter: " + str(self.normal_counter))
            print('Coords: ' + str(x) + ', ' + str(y))
            print('--------------------------')

            self.pub.publish(normal_coord)

            sm_flag = False
            while not rospy.is_shutdown():

                if(sm_flag):
                    sm_flag = False

                    if sm_command == "play":
                        print(sm_command)
                        sm_flag = True
                        sm_command = None
                        return 'play'

                    if(self.normal_counter < normal_times):
                        x = random.randrange(0,1000)
                        y = random.randrange(0,1000)
                        normal_coord = Point(x = x, y = y)
                        self.pub.publish(normal_coord)
                        self.normal_counter = self.normal_counter + 1
                        print("Times: " + str(normal_times))
                        print("Counter: " + str(self.normal_counter))
                        print('Coords: ' + str(x) + ', ' + str(y))
                        print('--------------------------')

                    else: return 'sleep'



# define state SLEEP
class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait'],
                                    input_keys = ['sleep_flag'],
                                    output_keys = ['sleep_coords'])

        self.pub = rospy.Publisher('move_coords', Point, queue_size=10)
        self.sub_flag = rospy.Subscriber('arrived', Bool, cb_flag)

    def execute(self, userdata):
        time.sleep(1)
        rospy.loginfo('Executing state SLEEP')

        global sm_flag

        sleep_x = 900
        sleep_y = 100
        sleep_coord = Point(x = sleep_x, y = sleep_y)

        self.pub.publish(sleep_coord)
        print("going to sleep")

        while not rospy.is_shutdown():

            if(sm_flag):
                print("Robot arrived to sleep")
                time.sleep(10)
                print("Robot woken")
                return 'wait'
        


# define state PLAY
class Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait'],
                                    input_keys = ['play_flag'],
                                    output_keys = ['play_coords'])

        self.pub_command = rospy.Publisher('gesture_request', String, queue_size=10)
        self.pub_coords = rospy.Publisher('move_coords', Point, queue_size=10)
        self.sub_flag = rospy.Subscriber('arrived', Bool, cb_flag)

        self.play_counter = 1

    def execute(self, userdata):
        time.sleep(1)
        rospy.loginfo('Executing state PLAY')

        play_times = random.randrange(1,5)

        global sm_flag

        man_x = 400
        man_y = 600
        man_coord = Point(x = man_x, y = man_y)

        sm_flag = False
        self.pub_coords.publish(man_coord)
        print("going towards man")
        print("lets play " + str(play_times) + " times")
        time.sleep(1)
        case = False

        while not rospy.is_shutdown():       

            if(sm_flag):
                sm_flag = False
                
                if(self.play_counter <= play_times):
                        if case == True:
                            self.pub_coords.publish(man_coord)
                            #self.play_counter = self.play_counter + 1
                            print('going towards man, time: ' + str(self.play_counter))
                            time.sleep(1)
                            case = False

                        elif case == False:
                            self.pub_command.publish("play")
                            print("going towards objective, time: " + str(self.play_counter))
                            self.play_counter = self.play_counter + 1
                            time.sleep(1)
                            case = True

                else: return 'wait'
    


sm_command = None
sm_flag = None

def cb_command(data):
    global sm_command
    sm_command = data.data


def cb_flag(data):
    global sm_flag
    sm_flag = data.data




# main
def main():

    global sm_command, sm_flag

    rospy.init_node('sm_assignment')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['COORDS'],
                            input_keys = ['sm_command', 'sm_flag'],
                            output_keys = ['sm_coords'])

    sm.userdata.sm_command = sm_command
    sm.userdata.sm_flag = sm_flag

    rate = rospy.Rate(10) # 10hz
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                                transitions={'play':'PLAY', 
                                            'sleep':'SLEEP',
                                            'coords': 'COORDS'},
                                remapping={ 'norm_command':'sm_command',
                                            'norm_flag': 'sm_flag',
                                            'norm_coords':'sm_coords'})

        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wait':'NORMAL'},
                               remapping={'sleep_coords': 'sm_coords',
                                            'sleep_flag': 'sm_flag'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'wait':'NORMAL'},
                               remapping={'play_coords': 'sm_coords',
                                            'play_flag': 'sm_flag'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    # Execute the state machine
    outcome = sm.execute()

    
    # Wait for ctrl-c to stop the application

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()