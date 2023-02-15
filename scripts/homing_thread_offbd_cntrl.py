#!/usr/bin/env python
# ROS python API
import time
import rospy
from threading import Thread, Lock
import numpy as np
from scipy.integrate import solve_ivp

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

cntSamplingTime = 5.0
v = 5.0
Rc = 1.0
k_alpha = 7.1
k_beta = 7.1
        
#inital Position of the quad
initialPos = Point(30.0, -25.0, 30.0)

#flag for checking whether the drone has reached home
isHome = False
landed = False

# Instantiate a setpoints message
setPoint = PositionTarget()
# set the flag to use position setpoints and yaw angle
setPoint.type_mask = int('010111111000', 2)
# LOCAL_NED
setPoint.coordinate_frame = 1

#initializing setpoint
setPoint.position.x = 0.0
setPoint.position.y = 0.0
setPoint.position.z = 0.0

mutexSetPoint = Lock()

def dubins3D(t,x, u_alpha, u_beta):
    alpha = x[3];
    beta  = x[4];

    dx = v*np.cos(beta)*np.cos(alpha)
    dy = v*np.cos(beta)*np.sin(alpha)
    dz = v*np.sin(beta)

    dalpha = u_alpha
    dbeta = u_beta

    dx_dt = [dx, dy, dz, dalpha, dbeta]
    return dx_dt


def sgn(x):
    if x>0:
        return 1.0
    else :
        return -1.0

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
    	rospy.wait_for_service('mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()


        # A Message for the previous local position of the drone
        self.prev_local_pos = Point(0.0, 0.0, 0.0)
        
        # A Message for the current local position of the drone
        self.local_pos = Point(2.0, 2.0, 3.0)

        self.alpha = 0
        self.beta = 0

        self.R = 0
        self.theta = 0
        self.phi = 0


# Callbacks

    ## local position callback
    def posCb(self, msg):
        
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

        #print "prev_pos", self.prev_local_pos
        #print "curr_pos", self.local_pos



    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        global isHome
            
        #print "prev_pos", self.prev_local_pos
        #print "curr_pos", self.local_pos
        dx = self.local_pos.x - self.prev_local_pos.x
        dy = self.local_pos.y - self.prev_local_pos.y
        dz = self.local_pos.z - self.prev_local_pos.z
        
        if dx == 0 or dy == 0 or dz == 0 :
            #print "Update error waiting for next : dx = ", dx, ", dy = ", dy, ", dz = ", dz 
            return
        self.R     = np.sqrt(self.local_pos.x**2 + self.local_pos.y**2 + self.local_pos.z**2)
        self.theta = np.arctan2(self.local_pos.y, self.local_pos.x)
        self.phi   = np.arctan(self.local_pos.z/np.sqrt(self.local_pos.x**2 + self.local_pos.y**2))
        
        if self.R < Rc :
            isHome = True

        if isHome:
            return
            
        self.alpha = np.arctan2( dy, dx )
        self.beta = np.arctan( dz/np.sqrt(dx**2 + dy**2) )

        
        dyaw  = self.alpha - self.theta - np.pi
        dyaw  = np.arctan2(np.sin(dyaw), np.cos(dyaw)) 
        dpitch = self.phi + self.beta
        #dpitch = np.arctan(np.sin(dpitch)/np.cos(dpitch))
        
        #print "alpha", self.alpha*180/np.pi, "theta", self.theta*180/np.pi, "beta", self.beta*180/np.pi, "phi", self.phi*180/np.pi, "dyaw", dyaw*180/np.pi, "dpitch", dpitch*180/np.pi 

        u_alpha = -k_alpha*sgn(dyaw)
        u_beta  = -k_beta*sgn(dpitch)
        
        t_span = np.linspace(0, 0.5, 100)
        x0 = [self.local_pos.x, self.local_pos.y, self.local_pos.z, self.alpha, self.beta]
        sol = solve_ivp(lambda t, x: dubins3D(t,x,u_alpha,u_beta),[t_span[0],t_span[-1]],x0, t_eval = t_span, rtol = 1e-5)
        
        mutexSetPoint.acquire()
        setPoint.position.x = sol.y[0,-1]
        setPoint.position.y = sol.y[1,-1]
        setPoint.position.z = sol.y[2,-1]
        mutexSetPoint.release()

        #print "Setpoint : ", setPoint.position.x, setPoint.position.y, setPoint.position.z
        self.prev_local_pos.x = self.local_pos.x
        self.prev_local_pos.y = self.local_pos.y
        self.prev_local_pos.z = self.local_pos.z



class pubThread (Thread):
    def __init__(self, threadID, name, initPos):
        Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        setPoint.position.x = initPos.x
        setPoint.position.y = initPos.y
        setPoint.position.z = initPos.z
        
        # ROS loop rate
        self.rate = rospy.Rate(20.0)
        
        # Setpoint publisher    
        self.setPointPub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    
    def run(self):
        global landed
        # ROS main loop
        while not rospy.is_shutdown() and  not landed:
            mutexSetPoint.acquire()
            self.setPointPub.publish(setPoint)
            mutexSetPoint.release()
            self.rate.sleep()
        #print "thread outside while"
        return


# Main function
def main():

    global isHome
    global landed

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(cntSamplingTime)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
       
    stateMsg = rospy.wait_for_message('/mavros/state', State, timeout = 0.3)
    cnt.stateCb(stateMsg)
    
    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    
    # # set in takeoff mode and takeoff to default altitude (3 m)
    # modes.setTakeoff()
    # rate.sleep()

    sendPos = pubThread(1, "Thread_1", initialPos)
    sendPos.start()
    
    rospy.sleep(2)
    

    # activate OFFBOARD mode
    modes.setOffboardMode()
    
    rospy.sleep(5)
    
    hom = raw_input("Start Homing ")
    
    #odomMessage = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout = 0.3)
    #cnt.posCb(odomMessage)
    
    #rospy.sleep(5)

    #odomMessage = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout = 0.3)
    #cnt.posCb(odomMessage)
    
    # ROS main loop
    while not rospy.is_shutdown() and not isHome:
    	#start_time = time.time() # for timing the function
        cnt.updateSp()
        #print("--- %s seconds ---" % (time.time() - start_time))
    	rate.sleep()

    #print "Outside while - main"

    if isHome :
        modes.setAutoLandMode()
        landed = True
    
    sendPos.join()
    return

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
