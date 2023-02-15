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
v = 2.0
Rc = 1.5
k_alpha = 2.1
k_beta = 2.1
        
#inital Position of the quad
initialPos = Point(30.0, -25.0, 30.0)

#flag for checking whether the drone has reached home
isHome = False
landed = False



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

# Flight modes class - have the functions to arm/disarm and setModes
# Made it into a class so that the general functionalities will be together for flight control
# This class is inherited to Drone class where all these functions are available - used as a interface
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self, droneId):
        self.droneId = droneId # Drone id is handy when multiple drones are used in the simulation or flight, use '' if not
        pass

    def setTakeoff(self):
    	rospy.wait_for_service('{}'.format(self.droneId)+'/mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('{}/mavros/cmd/takeoff'.format(self.droneId), mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('{}/mavros/cmd/arming'.format(self.droneId))
        try:
            armService = rospy.ServiceProxy('{}/mavros/cmd/arming'.format(self.droneId), mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('{}/mavros/cmd/arming'.format(self.droneId))
        try:
            armService = rospy.ServiceProxy('{}/mavros/cmd/arming'.format(self.droneId), mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    # modeList = ['STABILIZED', 'OFFBOARD','ALTCTL', 'POSCTL', 'AUTO.LAND']
    def setMode(self,mode):
        # print "In set mode : ", mode
        rospy.wait_for_service('{}/mavros/set_mode'.format(self.droneId))
        try:
            flightModeService = rospy.ServiceProxy('{}/mavros/set_mode'.format(self.droneId), mavros_msgs.srv.SetMode)
            flightModeService(custom_mode = mode)
        except rospy.ServiceException, e:
            print "Service set_mode call failed: %s."%e, "{} Mode could not be set.".format(mode)


# This is the thread class to be used for publihing the setpoints to drone when in offboard mode
# An object of this class is used in the Drone code and initiated when offboard mode is on
class pubThread (Thread):
    def __init__(self, threadID, name, droneId, isOffboard):
        Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.droneId = droneId
        self.isOffboard = isOffboard

        # Instantiate a setpoints message
        self.setPoint = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.setPoint.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.setPoint.coordinate_frame = 1

        #initializing setpoint
        self.setPoint.position.x = 0.0
        self.setPoint.position.y = 0.0
        self.setPoint.position.z = 0.0
        
        # ROS loop rate
        self.pubRate = rospy.Rate(20.0)
        
        self.mutexSetPoint = Lock()
        
        # Setpoint publisher    
        self.setPointPub = rospy.Publisher('{}/mavros/setpoint_raw/local'.format(self.droneId), PositionTarget, queue_size=1)
    
    def run(self):
        # print "in run"
        # ROS main loop
        while not rospy.is_shutdown() and self.isOffboard[0]:
            self.mutexSetPoint.acquire()
            self.setPointPub.publish(self.setPoint)
            self.mutexSetPoint.release()
            self.pubRate.sleep()
            #print "in thread : ",  self.droneState.mode, ",  prevMode = ", self.prevDroneMode
            #if self.droneState.mode != 'OFFBOARD' and self.prevDroneMode == 'OFFBOARD' :
            #    break
            #self.prevDroneMode = self.droneState.mode
        print "Exiting thread -> outside while"
        return



class Drone(fcuModes):
    def __init__(self, droneId, maxLinVel, maxYaw, maxPitch) :
        fcuModes.__init__(self, droneId)
        self.maxLinVel = maxLinVel
        self.maxYaw = maxYaw
        self.maxPitch = maxPitch
        self.isOffboard = [False]

        # Drone state
        self.state = State()


        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0.0)


        # Subscribe to drone state
        rospy.Subscriber('{}/mavros/state'.format(self.droneId), State, self.stateCb)

        # Subscribe to drone's local position
        rospy.Subscriber('{}/mavros/local_position/pose'.format(self.droneId), PoseStamped, self.posCb)

        self.sendPos = pubThread(0, '{}_publish_thread'.format(droneId), droneId, self.isOffboard)
        
        self.prevDroneMode = self.state.mode

# Methods
    def arm(self) :
        self.setArm()

    def disarm(self) :
        self.setDisarm(self)

    def modeSelect(self, mode):
        # print "in mode select "
        if mode == 'OFFBOARD' and not self.sendPos.is_alive():
            self.isOffboard[0] = True
            self.sendPos.start()
            # print "after run"
            rospy.sleep(2)
        else :
            self.isOffboard[0] = False
            rospy.sleep(2)

        self.setMode( mode)

    def getPos(self):
        return self.local_pos

    def setPos(self, pos):
        self.sendPos.mutexSetPoint.acquire()
        self.sendPos.setPoint.position.x = pos.x
        self.sendPos.setPoint.position.y = pos.y
        self.sendPos.setPoint.position.z = pos.z
        self.sendPos.mutexSetPoint.release()

    def autoUpdatePos(self) :
        pass

    
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
        if self.state.mode != 'OFFBOARD' and self.prevDroneMode == 'OFFBOARD' :
            self.isOffboard[0] = False

# Class for follower/interceptor
class followerDrone(Drone):
    # initialization method
    def __init__(self,droneId, maxLinVel, maxYaw, maxPitch):
        Drone.__init__(self, droneId, maxLinVel, maxYaw, maxPitch) 
        
        # A Message for the previous local position of the drone
        self.prev_local_pos = Point(0.0, 0.0, 0.0)
        

        self.alpha = 0
        self.beta = 0

        self.R = 0
        self.theta = 0
        self.phi = 0
        self.insideRc = False



    ## Update setpoint message
    def autoUpdatePos(self, targetPos):
                    
        #print "prev_pos", self.prev_local_pos
        #print "curr_pos", self.local_pos
        dx = self.local_pos.x - self.prev_local_pos.x
        dy = self.local_pos.y - self.prev_local_pos.y
        dz = self.local_pos.z - self.prev_local_pos.z
        
        if dx == 0 or dy == 0 or dz == 0 :
            #print "Update error waiting for next : dx = ", dx, ", dy = ", dy, ", dz = ", dz 
            return

        ex = targetPos.x - self.local_pos.x
        ey = targetPos.y - self.local_pos.y
        ez = targetPos.z - self.local_pos.z

        self.R     = np.sqrt(ex**2 + ey**2 + ez**2)
        self.theta = np.arctan2(ey, ex)
        self.phi   = np.arctan(ez/np.sqrt(ex**2 + ey**2))
        
        if self.R < Rc :
            insideRc = True
            self.modeSelect('POSCTL')
        else :
            insideRc = False

        if insideRc:
            return
            
        self.alpha = np.arctan2( dy, dx )
        self.beta = np.arctan( dz/np.sqrt(dx**2 + dy**2) )

        
        dyaw  = self.theta - self.alpha
        dyaw  = np.arctan2(np.sin(dyaw), np.cos(dyaw)) 
        dpitch = self.phi - self.beta
        #dpitch = np.arctan(np.sin(dpitch)/np.cos(dpitch))
        
        #print "alpha", self.alpha*180/np.pi, "theta", self.theta*180/np.pi, "beta", self.beta*180/np.pi, "phi", self.phi*180/np.pi, "dyaw", dyaw*180/np.pi, "dpitch", dpitch*180/np.pi 

        u_alpha = k_alpha*sgn(dyaw)
        u_beta  = k_beta*sgn(dpitch)
        
        t_span = np.linspace(0, 0.5, 100)
        x0 = [self.local_pos.x, self.local_pos.y, self.local_pos.z, self.alpha, self.beta]
        sol = solve_ivp(lambda t, x: dubins3D(t,x,u_alpha,u_beta),[t_span[0],t_span[-1]],x0, t_eval = t_span, rtol = 1e-5)
        
        self.sendPos.mutexSetPoint.acquire()
        self.sendPos.setPoint.position.x = sol.y[0,-1]
        self.sendPos.setPoint.position.y = sol.y[1,-1]
        self.sendPos.setPoint.position.z = sol.y[2,-1]
        self.sendPos.mutexSetPoint.release()

        #print "Setpoint : ", setPoint.position.x, setPoint.position.y, setPoint.position.z
        self.prev_local_pos.x = self.local_pos.x
        self.prev_local_pos.y = self.local_pos.y
        self.prev_local_pos.z = self.local_pos.z


# Class for target
class targetDrone(Drone):
    # initialization method
    def __init__(self,droneId, maxLinVel, maxYaw, maxPitch):
        Drone.__init__(self, droneId, maxLinVel, maxYaw, maxPitch) 
        self.t = 0.0



    ## Update setpoint message
    def autoUpdatePos(self):
        # global isHome
        
        self.t = (self.t + 1) % 201

        self.sendPos.mutexSetPoint.acquire()
        self.sendPos.setPoint.position.x = 4*np.cos(2*np.pi*(0.005)*self.t)
        self.sendPos.setPoint.position.y = 4*np.sin(2*np.pi*(0.005)*self.t)
        self.sendPos.setPoint.position.z = self.local_pos.z
        self.sendPos.mutexSetPoint.release()


# Main function
def main():

    global insideRc
    global landed

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)


    follower_initPos = Point(25.0, 25.0, 30.0)
    target_initPos = Point(4.0, 0.0, 5.0)
    followerObj = followerDrone('uav0', v, 5, 5)
    targetObj = targetDrone('uav1', v, 5, 5)
        
    # ROS loop rate
    rate = rospy.Rate(cntSamplingTime)

    while not followerObj.state.armed:
        followerObj.arm()
        rate.sleep()
    
    followerObj.modeSelect('OFFBOARD') 
    rospy.sleep(2)

    while not targetObj.state.armed:
        targetObj.arm()
        rate.sleep()
    
    targetObj.modeSelect('OFFBOARD') 
    rospy.sleep(2)
    
    followerObj.setPos(follower_initPos)
    rospy.sleep(4)

    targetObj.setPos(target_initPos)
    rospy.sleep(2)
    
    hom = raw_input("Start Homing : ")
    
    
    # ROS main loop
    while not rospy.is_shutdown() :
    	#start_time = time.time() # for timing the function
        followerObj.autoUpdatePos(targetObj.getPos())
        targetObj.autoUpdatePos()
        #print("--- %s seconds ---" % (time.time() - start_time))
    	rate.sleep()

    #print "Outside while - main"

    #if isDone :
    #    followerObj.setMode('AUTO.LAND')
    #    landed = True
    
    return

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
