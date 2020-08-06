#!/usr/bin/env python2

import numpy as np
from random import randint
import random
from math import *
import copy
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
import pickle 
#import sys

class Boatareascan():
    """
        Boat
    """
    def __init__(self, posx, posy, prevPos, initOri, psi_w):
        self.state=(posx, posy)
        self.next_state=(0.0, 0.0)
        self.prev_state=prevPos
        self.orientation=initOri
        self.psi_wind=psi_w
        self.reward=0
        self.initState=(posx, posy)
        self.initPrev=prevPos
        self.theta=[0, 0.78, 1.57, 2.35, 3.14, 3.92, 4.71, 5.50]
        self.Q_table={}
        for j in range(20):
            for i in range(20):
                c=(-50.0+i*5.0+5.0/2.0, 50.0-j*5.0-5.0/2.0);
                self.Q_table[c]=np.zeros(8)

        self.myPointValide=[]


    def reset(self):
        """
            Reset
        """
        self.state=(self.initState[0], self.initState[1])
        self.prev_state=(self.initPrev[0], self.initPrev[1])
        self.myPointValide[:]=[]


    """def take_action(self, eps):
        self.state
        act_possible=[x for x in range(8)]
        theta=[0, 0.78, 1.57, 2.35, 3.14, 3.92, 4.71, 5.50]
        for i in range(8):
            if (cos(self.psi_wind-theta[i])+cos(zeta))<0:
                act_possible.remove(i)
        if random.uniform(0, 1)<eps:
            act=random.choice(act_possible)
            return act
        else:
            act=np.argmax(self.Q_table[self.state])
            return act"""

    def take_action(self, eps):
        if random.uniform(0, 1)<eps:
            act=randint(0,7)
        else:
            act=np.argmax(self.Q_table[self.state])
        return act

    def getState(self, act):

        if (cos(self.psi_wind-self.theta[act])+cos(zeta))<0:
            self.next_state=(self.state[0], self.state[1])
            return 
        if act==0:
            new_point=(self.state[0]+5.0, self.state[1])
            
        if act==1:
            new_point=(self.state[0]+5.0, self.state[1]+5.0)

        if act==2:
            new_point=(self.state[0], self.state[1]+5.0)

        if act==3:
            new_point=(self.state[0]-5.0, self.state[1]+5.0)

        if act==4:
            new_point=(self.state[0]-5.0, self.state[1])

        if act==5:
            new_point=(self.state[0]-5.0, self.state[1]-5.0)

        if act==6:
            new_point=(self.state[0], self.state[1]-5.0)

        if act==7:
            new_point=(self.state[0]+5.0, self.state[1]-5.0)

        if arrInList(self.Q_table.keys(), new_point):
            self.next_state=new_point
        else:
            self.next_state=(self.state[0], self.state[1])
        return


    def get_rewards(self, PointValideByOtherBoat, act):
        self.reward=0
        if (cos(self.psi_wind-self.theta[act])+cos(zeta))<0:
            self.reward=-2.0
            return
        verif=False
        for L in PointValideByOtherBoat:
            if arrInList(L, self.next_state):
                verif=True
        if not verif:
            if abs(abs(self.orientation-self.theta[act])-0.0)<=0.01:
                self.reward=2.0

            if abs(abs(self.orientation-self.theta[act])-0.78)<=0.01:
                self.reward=1.0

            if abs(abs(self.orientation-self.theta[act])-1.57)<=0.01:
                self.reward=0.5

            if abs(abs(self.orientation-self.theta[act])-2.35)<=0.01:
                self.reward=0.25

            if abs(abs(self.orientation-self.theta[act])-3.14)<=0.01:
                self.reward=0.125

            if abs(abs(self.orientation-self.theta[act])-3.92)<=0.01:
                self.reward=0.25

            if abs(abs(self.orientation-self.theta[act])-4.71)<=0.01:
                self.reward=0.5

            if abs(abs(self.orientation-self.theta[act])-5.50)<=0.01:
                self.reward=1.0

        else:
            if abs(abs(self.orientation-self.theta[act])-0.0)<=0:
                self.reward=log(0.8)/3

            if abs(abs(self.orientation-self.theta[act])-0.78)<=0.01:
                self.reward=log(0.4)/3

            if abs(abs(self.orientation-self.theta[act])-1.57)<=0.01:
                self.reward=log(0.25)/3

            if abs(abs(self.orientation-self.theta[act])-2.35)<=0.01:
                self.reward=log(0.125)/3

            if abs(abs(self.orientation-self.theta[act])-3.14)<=0.01:
                self.reward=log(0.0625)/3

            if abs(abs(self.orientation-self.theta[act])-3.92)<=0.01:
                self.reward=log(0.125)/3

            if abs(abs(self.orientation-self.theta[act])-4.71)<=0.01:
                self.reward=log(0.25)/3

            if abs(abs(self.orientation-self.theta[act])-5.50)<=0.01:
                self.reward=log(0.4)/3
            
        return

        


    def train(self, eps, M):
        act1=self.take_action(eps)
        #print("state=", self.state)
        #print("action=", act1)
        self.getState(act1)
        if not arrInList(self.myPointValide, self.state):
            self.myPointValide.append(copy.deepcopy(self.state))
        self.get_rewards(M, act1)
        if not self.state==self.next_state:
            self.orientation=self.theta[act1]
        #print("reward=", self.reward)
        #print("next_state=", self.next_state)
        previous=(self.state[0], self.state[1])
        self.state=(self.next_state[0], self.next_state[1])
        act2=self.take_action(0.0)
        self.Q_table[previous][act1]=self.Q_table[previous][act1] + 0.1*(self.reward + 0.99*self.Q_table[self.state][act2]-self.Q_table[previous][act1])


def arrInList(L, arr):
    for a in L:
        if (a==arr):
            return True
    return False


def angle(a):
    return np.arctan2(a[1],a[0])

def sign(x):
    if x>0:
        return 1
    else:
        return -1

def norme(a):
    return sqrt(a[0]**2+a[1]**2)


def point_attract(m, theta, psi_w, a):
    vect=-2*(m-a)
    theta_bar=angle(vect)

    if cos(theta-theta_bar)>=0:
        ur=urmax*sin(theta-theta_bar)
    
    else:
        ur=urmax*sign(sin(theta-theta_bar))


    us=(pi/4)*(cos(psi_w-theta_bar)+1)

    return ur, us

def pose1callback(data):
    global m1
    m1[0]=data.x
    m1[1]=data.y

def pose2callback(data):
    global m2
    m2[0]=data.x
    m2[1]=data.y

def pose3callback(data):
    global m3
    m3[0]=data.x
    m3[1]=data.y

def pose4callback(data):
    global m4
    m4[0]=data.x
    m4[1]=data.y

def head1callback(data):
    global q_sail1
    q_sail1=(data.x, data.y, data.z, data.w)

def head2callback(data):
    global q_sail2
    q_sail2=(data.x, data.y, data.z, data.w)

def head3callback(data):
    global q_sail3
    q_sail3=(data.x, data.y, data.z, data.w)

def head4callback(data):
    global q_sail4
    q_sail4=(data.x, data.y, data.z, data.w)

def windcallback(data):
    global q_wind
    q_wind=(data.x, data.y, data.z, data.w)

def main():
    rospy.init_node('node_renf_learn')
    rospy.Subscriber("boat1_pose", Point, pose1callback)
    rospy.Subscriber("boat2_pose", Point, pose2callback)
    rospy.Subscriber("boat3_pose", Point, pose3callback)
    rospy.Subscriber("boat4_pose", Point, pose4callback)
    rospy.Subscriber("heading_boat1", Quaternion, head1callback)
    rospy.Subscriber("heading_boat2", Quaternion, head2callback)
    rospy.Subscriber("heading_boat3", Quaternion, head3callback)
    rospy.Subscriber("heading_boat4", Quaternion, head4callback)
    rospy.Subscriber("wind_angle", Quaternion, windcallback)
    com_servo1=rospy.Publisher("actuators1", Vector3, queue_size=10)
    com_servo2=rospy.Publisher("actuators2", Vector3, queue_size=10)
    com_servo3=rospy.Publisher("actuators3", Vector3, queue_size=10)
    com_servo4=rospy.Publisher("actuators4", Vector3, queue_size=10)
    pub_valid1=rospy.Publisher("center_point1", Vector3, queue_size=10)
    pub_valid2=rospy.Publisher("center_point2", Vector3, queue_size=10)
    pub_valid3=rospy.Publisher("center_point3", Vector3, queue_size=10)
    pub_valid4=rospy.Publisher("center_point4", Vector3, queue_size=10)

    rate = rospy.Rate(100)
    t0=rospy.get_time()
    while not rospy.is_shutdown():
        msg1, msg2, msg3, msg4=Vector3(), Vector3(), Vector3(), Vector3()
        msgCenter1, msgCenter2, msgCenter3, msgCenter4=Point(), Point(), Point(), Point()

        #rospy.loginfo("Good morning")
        theta1=euler_from_quaternion(q_sail1)[2]
        theta2=euler_from_quaternion(q_sail2)[2]
        theta3=euler_from_quaternion(q_sail3)[2]
        theta4=euler_from_quaternion(q_sail4)[2]

        tf=rospy.get_time()

        if tf-t0<0.3:
            rospy.loginfo("Do anything")
        else:

            act1=boat1.take_action(0.0)
            act2=boat2.take_action(0.0)
            act3=boat3.take_action(0.0)
            act4=boat4.take_action(0.0)

            boat1.getState(act1)
            boat2.getState(act2)
            boat3.getState(act3)
            boat4.getState(act4)

            a1=np.array(boat1.next_state)
            a2=np.array(boat2.next_state)
            a3=np.array(boat3.next_state)
            a4=np.array(boat4.next_state)

            ur1, us1=point_attract(m1, theta1, psi_w, a1)
            ur2, us2=point_attract(m2, theta2, psi_w, a2)
            ur3, us3=point_attract(m3, theta3, psi_w, a3)
            ur4, us4=point_attract(m4, theta4, psi_w, a4)

            if norme(m1-a1)<2.:
                boat1.state=boat1.next_state
                msgCenter1.x=a1[0]
                msgCenter1.y=a1[1]
                msgCenter1.z=1.0
                pub_valid1.publish(msgCenter1)
            if norme(m2-a2)<2.:
                boat2.state=boat2.next_state
                msgCenter2.x=a2[0]
                msgCenter2.y=a2[1]
                msgCenter2.z=2.0
                pub_valid2.publish(msgCenter2)
            if norme(m3-a3)<2.:
                boat3.state=boat3.next_state
                msgCenter3.x=a3[0]
                msgCenter3.y=a3[1]
                msgCenter3.z=3.0
                pub_valid3.publish(msgCenter3)
            if norme(m4-a4)<2.:
                boat4.state=boat4.next_state
                msgCenter4.x=a4[0]
                msgCenter4.y=a4[1]
                msgCenter4.z=4.0
                pub_valid4.publish(msgCenter4)

            msg1.x, msg1.y=ur1, us1
            msg2.x, msg2.y=ur2, us2
            msg3.x, msg3.y=ur3, us3
            msg4.x, msg4.y=ur4, us4
            com_servo1.publish(msg1)
            com_servo2.publish(msg2)
            com_servo3.publish(msg3)
            com_servo4.publish(msg4)
        rate.sleep()

    
if __name__ == '__main__':
    m1, m2, m3, m4=np.array([0,0]), np.array([0,0]), np.array([0,0]), np.array([0,0])
    q_sail1, q_sail2, q_sail3, q_sail4=(0,0,0,0), (0,0,0,0), (0,0,0,0), (0,0,0,0)
    q_wind=(0,0,0,0)
    psi_w=euler_from_quaternion(q_wind)[2]
    zeta=pi/4
    urmax=pi/4
    trainable=True
    boat1=Boatareascan(-47.5, 47.5, (-47.5, 52.5), -1.57, 0.0)
    boat2=Boatareascan(47.5, 47.5, (47.5, 52.5), -1.57, 0.0)
    boat3=Boatareascan(-47.5, -47.5, (-47.5, -52.5), 1.57, 0.0)
    boat4=Boatareascan(47.5, -47.5, (47.5, -52.5), 1.57, 0.0)

    if trainable:
        eps=1.0
        M=[boat1.myPointValide, boat2.myPointValide, boat3.myPointValide, boat4.myPointValide]
        for _ in range(100000):
            step=0
            while step<500:
                boat1.train(eps, M)
                boat2.train(eps, M)
                boat3.train(eps, M)
                boat4.train(eps, M)
                step+=1   

            eps=max(0.1, eps*0.9996)
            boat1.reset()
            boat2.reset()
            boat3.reset()
            boat4.reset()
        
        with open('/home/dembele/Workspace_Internship/devel/lib/sailboat/qtable', 'wb') as f:
            my_pickler = pickle.Pickler(f)
            my_pickler.dump(boat1.Q_table)
            my_pickler.dump(boat2.Q_table)
            my_pickler.dump(boat3.Q_table)
            my_pickler.dump(boat4.Q_table)
        f.close()

    else:
        with open('/home/dembele/Workspace_Internship/devel/lib/sailboat/qtable', 'rb') as f:
            my_depickler = pickle.Unpickler(f)
            boat1.Q_table=my_depickler.load()
            boat2.Q_table=my_depickler.load()
            boat3.Q_table=my_depickler.load()
            boat4.Q_table=my_depickler.load()
        f.close()
        print("boat1=", boat1.Q_table[(-47.5, 47.5)])
        print("boat2=", boat2.Q_table[(47.5, 47.5)])
        print("boat3=", boat3.Q_table[(-47.5, -47.5)])
        print("boat4=", boat4.Q_table[(47.5, -47.5)])

        main()

