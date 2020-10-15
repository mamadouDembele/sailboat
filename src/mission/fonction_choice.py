import numpy as np
import math
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn import neighbors
import rospy
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

data = pd.read_csv('choice.csv')

xtrain=np.matrix(data['psi_wind']).T
ytrain=np.matrix(data['fonction']).T

knn = neighbors.KNeighborsClassifier(n_neighbors=3)
knn.fit(xtrain, ytrain)

def callback(data):
	data.

def main():
    rospy.init_node('node_choice_fonction')
    pub = rospy.Publisher('fonc_to_choose', String, queue_size=10)
    rospy.Subscriber("wind_angle", Quaternion, callback)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
    	msg = String()
    	msg.data=0
    	pub.publish()
    	rospy.spinOnce()
  		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass


#!/usr/bin/env python2
# This Python file uses the following encoding: utf-8


from math import *
import copy
import random
import numpy as np
import tensorflow as tf
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion



class AreaScanBoat():
    """
        Boat
    """

    def __init__(self, case, psi_w, trainable=True):
        #init_position is an array
        self.state = np.array([case[0], case[1], psi_w])
        self.reward=0
        self.myPointValide=[]
        self.next_state=np.array([0.0,0.0,0.0])
        self.states=[]
        self.rewards=[]
        self.next_states=[]
        self.actions=[]
        self.tf_features= tf.placeholder(tf.float32, [None, 3]) #les données d'entrainements
        self.Qactions=tf.placeholder(tf.float32,[None, 8]) #les labels
        self.tf_mask=tf.placeholder(tf.float32,[None, 8])

        L=20 # Number of neurones on the first layers
        M=10 # Number of neurones on the second layers
        w1=tf.Variable(tf.random.truncated_normal([3,L], stddev=0.1)) # les poids du neurones
        self.w1=w1
        b1=tf.Variable(tf.zeros([L]))
        self.b1=b1
        w2=tf.Variable(tf.random.truncated_normal([L,M], stddev=0.1)) # les poids du neurones
        self.w2=w2
        b2=tf.Variable(tf.zeros([M]))
        self.b2=b2
        ws=tf.Variable(tf.random.truncated_normal([M,8], stddev=0.1)) # les poids du neurones
        self.ws=ws
        bs=tf.Variable(tf.zeros([8]))
        self.bs=bs

        # model
        z1 = tf.matmul(self.tf_features, self.w1) + self.b1 # fonction de pré-activation
        Y_hat1 = z1 # value predict by le model (fonction d'activation)

        z2 = tf.matmul(Y_hat1, self.w2) + self.b2
        Y_hat2 = tf.nn.relu(z2) # value predict by le model

        zs = tf.matmul(Y_hat2, self.ws) + self.bs
        self.y_hat =zs # value predict by le model



        cross_entropy=tf.reduce_mean(tf.math.multiply(tf.square(self.y_hat - self.Qactions), self.tf_mask))
        optimizer=tf.compat.v1.train.GradientDescentOptimizer(0.003)
        self.train_step=optimizer.minimize(cross_entropy)
        is_correct=tf.equal(tf.argmax(self.y_hat,1), tf.argmax(self.Qactions,1))
        self.accuracy=tf.reduce_mean(tf.cast(is_correct, tf.float32))

        # Define a section
        init=tf.global_variables_initializer()
        self.sess=tf.Session()
        self.sess.run(init)

        self.v=[]
        for j in range(20):
            for i in range(20):
                c=np.array([-50.0+i*5.0+5.0/2.0, 50.0-j*5.0-5.0/2.0]);
                self.v.append(c);
        self.numAreaScan=0.
        self.case = case

    def near_point(self, v):
        """
        point le plus proche 

        """
        next_point=np.array([0,0])
        distMinimal=float("inf")
        for i in range(len(v)):
            if distMinimal>norme(self.state[0:2]-v[i]):
                distMinimal=norme(self.state[0:2]-v[i])
                next_point=v[i]
        return next_point
                
    def near_list(self, v):
        """
            Les différents points atteingnables à partir d'une position bien donnée

        """
        L=[]
        v_copy=v[:]

        if (self.state[0]==47.5 and self.state[1]==47.5) or (self.state[0]==-47.5 and self.state[1]==47.5) or (self.state[0]==47.5 and self.state[1]==-47.5) or (self.state[0]==-47.5 and self.state[1]==-47.5):
            for i in range(3):
                next_point=self.near_point(v_copy)
                L.append(next_point)
                v_copy.remove(next_point)
            return L

        if (self.state[0]==47.5 and -47.5<self.state[1]<47.5) or (self.state[0]==-47.5 and -47.5<self.state[1]<47.5) or (self.state[1]==47.5 and -47.5<self.state[0]<47.5) or (self.state[1]==-47.5 and -47.5<self.state[0]<47.5):
            for i in range(5):
                next_point=self.near_point(v_copy)
                L.append(next_point)
                v_copy.remove(next_point)
            return L

        else:
            for i in range(8):
                next_point=self.near_point(v_copy)
                L.append(next_point)
                v_copy.remove(next_point)
            return L  

    def Eightpoint_in_face(self):
        v=self.point_in_face()
        #print(v)
        L=[]
        for i in range(8):
            next_point=self.near_point(v)
            #print(next_point)
            removearray(v, next_point)
            L.append(next_point)
        return L

    def take_action(self, eps):
        st=self.state
        #print("st=", st)
        act_possible=[x for x in range(8)]
        if (st[0:2]==np.array([-47.5,67.5])).all() or (st[0:2]==np.array([47.5,67.5])).all() or (st[0:2]==np.array([-47.5,-67.5])).all() or (st[0:2]==np.array([47.5,-67.5])).all():
            #print("yes")
            v_face=self.Eightpoint_in_face()
            #print("v_face= ",v_face)
            for i in range(len(v_face)):
                theta=angle(v_face[i]-st[0:2])
                #print("theta=", (cos(st[2]-theta)+cos(zeta)), st[2])
                if (cos(st[2]-theta)+cos(zeta))<0 and abs((cos(st[2]-theta)+cos(zeta)))>0.2:
                    act_possible.remove(i)
            if random.uniform(0, 1) < eps:
                #print("act_possible=", act_possible)
                act=random.choice(act_possible)
                return act
            else:
                #utiliser les reseaux de neurones
                result=self.sess.run(self.y_hat, feed_dict={self.tf_features: st.reshape(1,3)})
                liste=(result.reshape(8)).tolist()
                #print("Liste=", liste)
                act=liste.index(max(liste))
                return act
        else:
            if st[2]<0:
                psi_w=st[2]+2*pi
            if st[2]>6.28:
                psi_w=st[2]-2*pi
            else:
                psi_w=st[2]
            theta=[0, 0.78, 1.57, 2.35, 3.14, 3.92, 4.71, 5.50]
            for i in range(8):
                if (cos(psi_w-theta[i])+cos(zeta))<0:
                    act_possible.remove(i)
            if random.uniform(0, 1)<eps:
                #print("act_possible=", act_possible)
                act=random.choice(act_possible)
                return act
            else:
                #utiliser les reseaux de neurones
                result=self.sess.run(self.y_hat, feed_dict={self.tf_features: st.reshape(1,3)})
                liste=(result.reshape(8)).tolist()
                #print("Liste=", liste)
                act=liste.index(max(liste))
                return act


    def point_in_face(self):
        v=[]
        if (self.state[0:2]==np.array([-47.5,67.5])).all() or (self.state[0:2]==np.array([47.5,67.5])).all():
            for i in range(20):
                c=np.array([-50+i*5+5.0/2.0, 47.5])
                v.append(c)
            return v
        if (self.state[0:2]==np.array([-47.5,-67.5])).all() or (self.state[0:2]==np.array([47.5,-67.5])).all():
            for i in range(20):
                c=np.array([-50+i*5+5.0/2.0, -47.5])
                v.append(c)
            return v


    def reset(self):
        self.state[0:2]=self.case

    def all_reset(self):
        self.state[0:2]=self.case
        self.reward=0
        self.myPointValide=[]
        self.next_state=np.array([0.0,0.0,0.0])
        self.states=[]
        self.rewards=[]
        self.next_states=[]
        self.actions=[]

    def getState(self, act):
        st=self.state[0:2]
        if (st[0:2]==np.array([-47.5,67.5])).all() or (st[0:2]==np.array([47.5,67.5])).all() or (st[0:2]==np.array([-47.5,-67.5])).all() or (st[0:2]==np.array([47.5,-67.5])).all():
            v_face=self.Eightpoint_in_face()
            self.next_state[0:2]=v_face[act]
            self.next_state[2]=self.state[2]
            return
        else:
            if act==0:
                new_point=np.array([self.state[0]+5.0, self.state[1]])
                #print("new_point=", new_point)
                if arrInList(self.v, new_point):
                    self.next_state[0:2]=new_point
                    self.next_state[2]=self.state[2]
                else:
                    self.next_state=self.state
                return

            if act==1:
                new_point=np.array([self.state[0]+5.0, self.state[1]+5.0])
                #print("new_point=", new_point)
                if arrInList(self.v, new_point):
                    self.next_state[0:2]=new_point
                    self.next_state[2]=self.state[2]
                else:
                    self.next_state=self.state
                return

            if act==2:
                new_point=np.array([self.state[0], self.state[1]+5.0])
                #print("new_point=", new_point)
                if arrInList(self.v, new_point):
                    self.next_state[0:2]=new_point
                    self.next_state[2]=self.state[2]
                else:
                    self.next_state=self.state
                return

            if act==3:
                new_point=np.array([self.state[0]-5.0, self.state[1]+5.0])
                #print("new_point=", new_point)
                if arrInList(self.v, new_point):
                    self.next_state[0:2]=new_point
                    self.next_state[2]=self.state[2]
                else:
                    self.next_state=self.state
                return

            if act==4:
                new_point=np.array([self.state[0]-5.0, self.state[1]])
                #print("new_point=", new_point)
                if arrInList(self.v, new_point):
                    self.next_state[0:2]=new_point
                    self.next_state[2]=self.state[2]
                else:
                    self.next_state=self.state
                return

            if act==5:
                new_point=np.array([self.state[0]-5.0, self.state[1]-5.0])
                #print("new_point=", new_point)
                if arrInList(self.v, new_point):
                    self.next_state[0:2]=new_point
                    self.next_state[2]=self.state[2]
                else:
                    self.next_state=self.state
                return

            if act==6:
                new_point=np.array([self.state[0], self.state[1]-5.0])
                #print("new_point=", new_point)
                if arrInList(self.v, new_point):
                    self.next_state[0:2]=new_point
                    self.next_state[2]=self.state[2]
                else:
                    self.next_state=self.state
                return

            if act==7:
                new_point=np.array([self.state[0]+5.0, self.state[1]-5.0])
                #print("new_point=", new_point)
                if arrInList(self.v, new_point):
                    self.next_state[0:2]=new_point
                    self.next_state[2]=self.state[2]
                else:
                    self.next_state=self.state
                return


    def get_rewards(self, PointValideByOtherBoat):
        #print(self.myPointValide)
        if arrInList(self.myPointValide, self.next_state[0:2]):
            self.reward=-1
            return
        n=1
        verif=False
        for L in PointValideByOtherBoat:
            #print("L=", L)
            if arrInList(L, self.next_state[0:2]):
                n+=1
                verif=True
        if verif==True:
            self.reward=1/n
            return

        else:
            self.reward=1.
            return


    def train(self):

        states=trans_list_np(self.states)
        #print("states=", states)
        actions=trans_list_np(self.actions)
        #print("actions=", actions)
        next_states=trans_list_np(self.next_states)
        #print("next_states=",next_states)
        rewards=trans_list_np(self.rewards)
        #print("rewards=", rewards)

        allQmax=self.sess.run(self.y_hat, feed_dict={self.tf_features: next_states})

        #print("allQmax=", allQmax.shape)


        target=rewards+0.99*np.amax(allQmax, 1).reshape(allQmax.shape[0], 1)
        targets=actions*target
        #print("targets=", targets[10,:])

        batch_size = 100
        k=0
        #print("states.shape[0]=", states.shape[0])
        while batch_size<states.shape[0]:
            batch_states=states[k:batch_size, :]
            batch_targets=targets[k:batch_size, :]
            batch_mask=actions[k:batch_size, :]
            #print("batch_size=", batch_size)
            self.sess.run(self.train_step, feed_dict={self.tf_features: batch_states, self.Qactions: batch_targets, self.tf_mask: batch_mask})
            if batch_size%1000==0:
                print(self.sess.run(self.accuracy, feed_dict={self.tf_features: batch_states, self.Qactions: batch_targets, self.tf_mask: batch_mask}))
            k=batch_size
            batch_size+=100

def trans_list_np(L):
    #print("L=", L)
    v=np.zeros((len(L),np.size(L[0])))
    for i in range(len(L)):
        for j in range(np.size(L[0])):
            v[i,j]=L[i][j]
    return v

def removearray(L,arr):
    ind = 0
    size = len(L)
    while ind != size and not np.array_equal(L[ind],arr):
        ind += 1
    if ind != size:
        L.pop(ind)
    else:
        raise ValueError('array not found in list.')

def arrInList(L, arr):
    for a in L:
        if (a==arr).all():
            return True
    return False

def insertstate(L, arr, i):
    L.insert(i,copy.deepcopy(arr))
        
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

    rate = rospy.Rate(100)
    t0=rospy.get_time()
    while not rospy.is_shutdown():
        msg1, msg2, msg3, msg4=Vector3(), Vector3(), Vector3(), Vector3()

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
            rospy.loginfo("action=%f", act3)
            act4=boat4.take_action(0.0)

            boat1.getState(act1)
            boat2.getState(act2)
            boat3.getState(act3)
            rospy.loginfo("boat_position=%f,%f", boat3.state[0], boat3.state[1])
            boat4.getState(act4)

            a1=boat1.next_state[0:2]
            a2=boat2.next_state[0:2]
            a3=boat3.next_state[0:2]
            a4=boat4.next_state[0:2]

            ur1, us1=point_attract(m1, theta1, psi_w, a1)
            ur2, us2=point_attract(m2, theta2, psi_w, a2)
            ur3, us3=point_attract(m3, theta3, psi_w, a3)
            ur4, us4=point_attract(m4, theta4, psi_w, a4)

            if norme(m1-a1)<1.:
                boat1.state=boat1.next_state
            if norme(m2-a2)<1.:
                boat2.state=boat2.next_state
            if norme(m3-a3)<1.:
                boat3.state=boat3.next_state
            if norme(m4-a4)<1.:
                boat4.state=boat4.next_state

            msg1.x, msg1.y=ur1, us1
            msg2.x, msg2.y=ur2, us2
            msg3.x, msg3.y=ur3, us3
            msg4.x, msg4.y=ur4, us4
            com_servo1.publish(msg1)
            com_servo2.publish(msg2)
            com_servo3.publish(msg3)
            com_servo4.publish(msg4)
        #rospy.spin()
        rate.sleep()

    
if __name__ == '__main__':
    m1, m2, m3, m4=np.array([0,0]), np.array([0,0]), np.array([0,0]), np.array([0,0])
    q_sail1, q_sail2, q_sail3, q_sail4=(0,0,0,0), (0,0,0,0), (0,0,0,0), (0,0,0,0)
    q_wind=(0,0,0,0)
    psi_w=euler_from_quaternion(q_wind)[2]
    print("psi_w=", psi_w)
    theta=pi/3

    L=[]

    trainable=False
    init=tf.initialize_all_variables()
    zeta=pi/4
    urmax=pi/4

    boat1=AreaScanBoat(np.array([-47.5,67.5]), psi_w, init)
    boat2=AreaScanBoat(np.array([47.5,67.5]), psi_w, init)
    boat3=AreaScanBoat(np.array([-47.5,-67.5]), psi_w, init)
    boat4=AreaScanBoat(np.array([47.5,-67.5]), psi_w, init)
    
    if trainable:
        eps=1.0
        M=[boat1.myPointValide, boat2.myPointValide, boat3.myPointValide, boat4.myPointValide]
        for i in range(1,150):
            step=0
            while step<25000:
                act1=boat1.take_action(eps)
                act2=boat2.take_action(eps)
                act3=boat3.take_action(eps)
                act4=boat4.take_action(eps)

                mask1=np.zeros(8)
                mask2=np.zeros(8)
                mask3=np.zeros(8)
                mask4=np.zeros(8)

                mask1[act1]=1
                mask2[act2]=1
                mask3[act3]=1
                mask4[act4]=1


                boat1.getState(act1)
                boat2.getState(act2)
                boat3.getState(act3)
                boat4.getState(act4)

                boat1.get_rewards(M)
                boat2.get_rewards(M)
                boat3.get_rewards(M)
                boat4.get_rewards(M)

                indice=random.randint(0,len(boat1.states))

                insertstate(boat1.states, boat1.state, indice)
                insertstate(boat1.rewards, np.array([boat1.reward]), indice)
                insertstate(boat1.next_states, boat1.next_state, indice)
                insertstate(boat1.actions, mask1, indice)

                insertstate(boat2.states, boat2.state, indice)
                insertstate(boat2.rewards, np.array([boat2.reward]), indice)
                insertstate(boat2.next_states, boat2.next_state, indice)
                insertstate(boat2.actions, mask2, indice)

                insertstate(boat3.states, boat3.state, indice)
                insertstate(boat3.rewards, np.array([boat3.reward]), indice)
                insertstate(boat3.next_states, boat3.next_state, indice)
                insertstate(boat3.actions, mask3, indice)

                insertstate(boat4.states, boat4.state, indice)
                insertstate(boat4.rewards, np.array([boat4.reward]), indice)
                insertstate(boat4.next_states, boat4.next_state, indice)
                insertstate(boat4.actions, mask4, indice)

                if len(boat1.states)>20000:
                    boat1.states.pop()
                    boat2.states.pop()
                    boat3.states.pop()
                    boat4.states.pop()

                #print("state=", copy.deepcopy(boat1.state))
                #print("-------------------------")
                #print("action=", act1)
                #print("-------------------------")
                #print("next_state=", copy.deepcopy(boat1.next_state))
                #print("-------------------------")
                #print("reward=", copy.deepcopy(boat1.reward))
                #print("####################################")

                boat1.state=copy.deepcopy(boat1.next_state)
                boat2.state=copy.deepcopy(boat2.next_state)
                boat3.state=copy.deepcopy(boat3.next_state)
                boat4.state=copy.deepcopy(boat4.next_state)

                if not arrInList(boat1.myPointValide, boat1.state[0:2]):
                    boat1.myPointValide.append(copy.deepcopy(boat1.state[0:2]))

                if not arrInList(boat2.myPointValide, boat2.state[0:2]):
                    boat2.myPointValide.append(copy.deepcopy(boat2.state[0:2]))

                if not arrInList(boat3.myPointValide, boat3.state[0:2]):
                    boat3.myPointValide.append(copy.deepcopy(boat3.state[0:2]))

                if not arrInList(boat4.myPointValide, boat4.state[0:2]):
                    boat4.myPointValide.append(copy.deepcopy(boat4.state[0:2]))

                #print("M=", M)
                #print("--------------------------------------")
                #print("--------------------------------------")

                step=step+1

                if step%500==0:
                    boat1.reset()
                    boat2.reset()
                    boat3.reset()
                    boat4.reset()
                    boat1.state[2]+=0.1
                    boat2.state[2]+=0.1
                    boat3.state[2]+=0.1
                    boat4.state[2]+=0.1
                    #print("reset position")

            print("new train")
            print("accuracy boat1: acc=")
            boat1.train()
            print("--------------------------------------")
            print("--------------------------------------")
            print("accuracy boat2: acc=")
            boat2.train()
            print("--------------------------------------")
            print("--------------------------------------")
            print("accuracy boat3: acc=")
            boat3.train()
            print("--------------------------------------")
            print("--------------------------------------")
            print("accuracy boat4: acc=")
            boat4.train()
            print("--------------------------------------")
            print("--------------------------------------")
            eps=max(0.1, eps*0.99)
            boat1.all_reset()
            boat2.all_reset()
            boat3.all_reset()
            boat4.all_reset()
    #print("finish")
    print("Variable boat1: w1=", boat1.sess.run(boat1.w1))
    print("Variable boat1: b1=", boat1.sess.run(boat1.b1))
    print("Variable boat1: w2=", boat1.sess.run(boat1.w2))
    print("Variable boat1: b2=", boat1.sess.run(boat1.b2))
    print("Variable boat1: ws=", boat1.sess.run(boat1.ws))
    print("Variable boat1: bs=", boat1.sess.run(boat1.bs))
    print("--------------------------------------")
    print("--------------------------------------")

    print("Variable boat2: w1=", boat2.sess.run(boat2.w1))
    print("Variable boat2: b1=", boat2.sess.run(boat2.b1))
    print("Variable boat2: w2=", boat2.sess.run(boat2.w2))
    print("Variable boat2: b2=", boat2.sess.run(boat2.b2))
    print("Variable boat2: ws=", boat2.sess.run(boat2.ws))
    print("Variable boat2: bs=", boat2.sess.run(boat2.bs))
    print("--------------------------------------")
    print("--------------------------------------")

    print("Variable boat3: w1=", boat3.sess.run(boat3.w1))
    print("Variable boat3: b1=", boat3.sess.run(boat3.b1))
    print("Variable boat3: w2=", boat3.sess.run(boat3.w2))
    print("Variable boat3: b2=", boat3.sess.run(boat3.b2))
    print("Variable boat3: ws=", boat3.sess.run(boat3.ws))
    print("Variable boat3: bs=", boat3.sess.run(boat3.bs))
    print("--------------------------------------")
    print("--------------------------------------")

    print("Variable boat4: w1=", boat4.sess.run(boat4.w1))
    print("Variable boat4: b1=", boat4.sess.run(boat4.b1))
    print("Variable boat4: w2=", boat4.sess.run(boat4.w2))
    print("Variable boat4: b2=", boat4.sess.run(boat4.b2))
    print("Variable boat4: ws=", boat4.sess.run(boat4.ws))
    print("Variable boat4: bs=", boat4.sess.run(boat4.bs))

    boat1.w1.load(np.array([[-0.05061387,  0.0635446 ,  0.08334445, -0.03647125,  0.1600388 ,
        -0.01790873,  0.08106504,  0.09668469,  0.04725571, -0.06825691,
         0.01482597, -0.09582601, -0.03524664,  0.01770774, -0.0252922 ,
         0.12779436,  0.00983142,  0.07794499,  0.07951044, -0.03240181],
       [ 0.0747769 ,  0.01962232, -0.00498397, -0.01462256, -0.00896931,
         0.08579919, -0.05664384, -0.02741529,  0.0475062 , -0.04771959,
        -0.01508099, -0.00281398,  0.0761103 , -0.01463857, -0.15292723,
        -0.06687266, -0.08100671, -0.05013308, -0.00367015, -0.02328208],
       [-0.02991575,  0.08438978,  0.14253409,  0.05232048,  0.0526805 ,
        -0.05783519,  0.06274707, -0.05400972,  0.04297901, -0.06835347,
        -0.02993434,  0.07127781, -0.00751894,  0.13725094, -0.06871824,
        -0.05778428, -0.03155334,  0.01595913,  0.05628278,  0.01143661]]), boat1.sess)
    boat1.b1.load(np.array([-5.5278098e-04, -5.4755236e-04,  1.5291914e-03,  2.4152007e-03,
        3.4045827e-04,  5.0109444e-04, -1.0376784e-03, -1.1304177e-03,
        1.1564813e-03, -2.5943894e-04,  1.4677994e-04,  6.1634637e-05,
        6.1617116e-04, -4.3177391e-05, -1.5090638e-03, -3.4341495e-04,
        1.4042920e-04, -1.4617095e-04,  7.2655478e-04,  1.2303228e-04]), boat1.sess)
    boat1.w2.load(np.array([[ 0.08179604,  0.12889849,  0.01459162,  0.15353481,  0.07066438,
         0.14254715,  0.09730428,  0.07372432,  0.02830476, -0.10769924],
       [-0.09882501, -0.21588443,  0.00252198,  0.01120125,  0.02010744,
         0.16345738,  0.13873528,  0.03590312, -0.07354151, -0.04361519],
       [ 0.17263702, -0.00076009,  0.05070511, -0.02003347,  0.00192757,
        -0.10581927,  0.10908511, -0.0943712 , -0.01919077, -0.09469771],
       [ 0.1479538 , -0.0352648 , -0.13177465, -0.12804392,  0.06835857,
        -0.12429922, -0.08123774,  0.08151598, -0.03813426, -0.02258747],
       [ 0.01730583, -0.01355855, -0.13490717, -0.0159135 ,  0.04339125,
        -0.04846092,  0.07085599, -0.05890004, -0.00330229, -0.07642433],
       [ 0.04795824,  0.01078648, -0.01557973, -0.02803561,  0.05479928,
        -0.08635361,  0.07235117,  0.13826892,  0.1786952 , -0.02981706],
       [-0.07355693,  0.12736917, -0.07622089,  0.00174216, -0.04984892,
        -0.00373011, -0.06953683, -0.1634731 ,  0.04309179, -0.10796013],
       [-0.02510737,  0.04304734,  0.04549577,  0.09837236, -0.00914442,
        -0.0800937 ,  0.00429297,  0.14235334,  0.07398596,  0.09683595],
       [ 0.14254612,  0.11153111, -0.00215818, -0.01951156, -0.09517043,
        -0.1300698 , -0.05836867,  0.00720733, -0.12088514,  0.08807327],
       [-0.08319355, -0.03272077, -0.01401916, -0.1513529 , -0.01837766,
         0.19066294,  0.12279797,  0.03343097,  0.01730869,  0.03228914],
       [ 0.03916679, -0.08706263,  0.05455857,  0.03395054,  0.07826737,
         0.02574764,  0.04577912,  0.07928108,  0.18963645,  0.06124246],
       [ 0.00892604, -0.02023382,  0.00168369, -0.03145244, -0.05496241,
        -0.02377947,  0.10306902,  0.01514295, -0.07171042,  0.1469736 ],
       [ 0.00667459, -0.0466908 ,  0.07510148, -0.07513719, -0.05892919,
        -0.06499082, -0.04391302,  0.00314562, -0.01157301,  0.11541908],
       [ 0.05841777,  0.04520397,  0.05247743,  0.07911062, -0.02333828,
        -0.1028618 , -0.17213435, -0.01833119, -0.02227681,  0.06106891],
       [-0.14973037,  0.03185097,  0.09164657,  0.00836281,  0.01608057,
        -0.0935525 , -0.07907794,  0.00189891, -0.03830311, -0.02139051],
       [-0.04068473, -0.03676955, -0.00933601, -0.02140675,  0.05958175,
         0.00290219,  0.07124941, -0.11458734,  0.05397745,  0.09489571],
       [ 0.04815051,  0.01118695, -0.13069269,  0.06570847, -0.02131981,
         0.00156652,  0.14862137,  0.15085497,  0.03841797, -0.14936744],
       [ 0.03499126, -0.00408242,  0.04018087, -0.01042685, -0.07164947,
         0.05486016,  0.10693848,  0.06138746,  0.01691828,  0.12677433],
       [ 0.09882205,  0.06091012, -0.09273826, -0.04284844, -0.11623225,
         0.08691154,  0.0214277 ,  0.02621597, -0.07267294,  0.09080477],
       [ 0.06249048,  0.03723095, -0.00327159,  0.12019026,  0.01203324,
        -0.04519994, -0.0446975 , -0.05247317,  0.03305733, -0.0835791 ]]), boat1.sess)

    boat1.b2.load(np.array([ 0.00787756, -0.00416282, -0.00154234, -0.00422705, -0.00054387,
       -0.00103653, -0.00118021, -0.00091096, -0.00077305, -0.00085019]), boat1.sess)
    boat1.ws.load(np.array([[ 0.10296779,  0.10221453,  0.10108826,  0.10254063,  0.10311031,
         0.10264011,  0.10150097,  0.10254077],
       [-0.00536408, -0.08171212, -0.03170898,  0.02428698, -0.04396623,
        -0.01311495, -0.04990034,  0.14687024],
       [-0.04989687,  0.11566404, -0.10690696, -0.03827113,  0.08834455,
         0.06144179, -0.05634335, -0.05530525],
       [-0.00805688,  0.08480933, -0.09685034,  0.00028341,  0.12964498,
        -0.04172666, -0.03184964, -0.06742167],
       [-0.0392306 , -0.15622528,  0.07106741,  0.00191832, -0.04362287,
         0.01493575, -0.10481943,  0.08876511],
       [ 0.04688466,  0.02858979, -0.10574836,  0.03086441,  0.00386964,
         0.09180404, -0.10376079,  0.04922099],
       [ 0.18565801, -0.03762325,  0.05332333,  0.00446192, -0.0470752 ,
        -0.14082208, -0.0380821 ,  0.08383731],
       [ 0.08900791,  0.04631068,  0.0503411 ,  0.02899382, -0.05025424,
        -0.18291274, -0.03664696,  0.05879568],
       [ 0.16184668, -0.00981211,  0.01722343, -0.05167339,  0.03607873,
        -0.01034329, -0.10158596, -0.01317146],
       [-0.062678  , -0.06715318,  0.00177502,  0.05204096, -0.0589746 ,
         0.08875277,  0.10558745,  0.07263686]]), boat1.sess)
    boat1.bs.load(np.array([0.02598252, 0.04984469, 0.03961275, 0.03644312, 0.01197934,
       0.02545067, 0.05810546, 0.02533981]), boat1.sess)
    if not trainable:
        main()