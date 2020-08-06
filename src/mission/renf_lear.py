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

    def __init__(self, case, psi_w):
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

        L=15 # Number of neurones on the first layers
        M=15 # Number of neurones on the second layers

        w1=tf.Variable(tf.random.truncated_normal([3,L], stddev=0.1))
        self.w1=w1
        self.w1_copy=w1
        b1=tf.Variable(tf.zeros([L]))
        self.b1=b1
        self.b1_copy=b1
        w2=tf.Variable(tf.random.truncated_normal([L,M], stddev=0.1)) # les poids du neurones
        self.w2=w2
        self.w2_copy=w2
        b2=tf.Variable(tf.zeros([M]))
        self.b2=b2
        self.b2_copy=b2
        ws=tf.Variable(tf.random.truncated_normal([M,8], stddev=0.1)) # les poids du neurones
        self.ws=ws
        self.ws_copy=ws
        bs=tf.Variable(tf.zeros([8]))
        self.bs=bs
        self.bs_copy=bs

        # model
        z1 = tf.matmul(self.tf_features, self.w1) + self.b1 # fonction de pré-activation
        Y_hat1 = z1 # value predict by le model (fonction d'activation)

        z2 = tf.matmul(Y_hat1, self.w2) + self.b2
        Y_hat2 = tf.nn.relu(z2) # value predict by le model

        zs = tf.matmul(Y_hat2, self.ws) + self.bs
        self.y_hat=zs

        #copy of neuronal
        z1_copy = tf.matmul(self.tf_features, self.w1_copy) + self.b1_copy # fonction de pré-activation
        Y_hat1_copy = z1_copy # value predict by le model (fonction d'activation)

        z2_copy = tf.matmul(Y_hat1_copy, self.w2_copy) + self.b2_copy
        Y_hat2_copy = tf.nn.relu(z2_copy) # value predict by le model

        zs_copy = tf.matmul(Y_hat2_copy, self.ws_copy) + self.bs_copy
        self.y_hat_copy =zs_copy # value predict by le model



        cross_entropy=tf.reduce_mean(tf.square(self.y_hat - self.Qactions))
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


    def take_action(self, eps):
        st=self.state
        act_possible=[x for x in range(8)]
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
            act=liste.index(max(liste))
            return act



    def reset(self):
        self.state[0:2]=self.case

    def copy_model_parameter(self):
        self.w1_copy.load(self.sess.run(self.w1), self.sess)
        self.b1_copy.load(self.sess.run(self.b1), self.sess)
        self.w2_copy.load(self.sess.run(self.w2), self.sess)
        self.b2_copy.load(self.sess.run(self.b2), self.sess)
        self.ws_copy.load(self.sess.run(self.ws), self.sess)
        self.bs_copy.load(self.sess.run(self.bs), self.sess)


    def getState(self, act):
        if act==0:
            new_point=np.array([self.state[0]+5.0, self.state[1]])
            if arrInList(self.v, new_point):
                self.next_state[0:2]=new_point
                self.next_state[2]=self.state[2]
            else:
                self.next_state=self.state
            return

        if act==1:
            new_point=np.array([self.state[0]+5.0, self.state[1]+5.0])
            if arrInList(self.v, new_point):
                self.next_state[0:2]=new_point
                self.next_state[2]=self.state[2]
            else:
                self.next_state=self.state
            return

        if act==2:
            new_point=np.array([self.state[0], self.state[1]+5.0])
            if arrInList(self.v, new_point):
                self.next_state[0:2]=new_point
                self.next_state[2]=self.state[2]
            else:
                self.next_state=self.state
            return

        if act==3:
            new_point=np.array([self.state[0]-5.0, self.state[1]+5.0])
            if arrInList(self.v, new_point):
                self.next_state[0:2]=new_point
                self.next_state[2]=self.state[2]
            else:
                self.next_state=self.state
            return

        if act==4:
            new_point=np.array([self.state[0]-5.0, self.state[1]])
            if arrInList(self.v, new_point):
                self.next_state[0:2]=new_point
                self.next_state[2]=self.state[2]
            else:
                self.next_state=self.state
            return

        if act==5:
            new_point=np.array([self.state[0]-5.0, self.state[1]-5.0])
            if arrInList(self.v, new_point):
                self.next_state[0:2]=new_point
                self.next_state[2]=self.state[2]
            else:
                self.next_state=self.state
            return

        if act==6:
            new_point=np.array([self.state[0], self.state[1]-5.0])
            if arrInList(self.v, new_point):
                self.next_state[0:2]=new_point
                self.next_state[2]=self.state[2]
            else:
                self.next_state=self.state
            return

        if act==7:
            new_point=np.array([self.state[0]+5.0, self.state[1]-5.0])
            if arrInList(self.v, new_point):
                self.next_state[0:2]=new_point
                self.next_state[2]=self.state[2]
            else:
                self.next_state=self.state
            return


    def get_rewards(self, PointValideByOtherBoat):
        if arrInList(self.myPointValide, self.next_state[0:2]):
            self.reward=-1
            return
        n=1
        verif=False
        for L in PointValideByOtherBoat:
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
        actions=trans_list_np(self.actions)
        next_states=trans_list_np(self.next_states)
        rewards=trans_list_np(self.rewards)

        allQmax=self.sess.run(self.y_hat, feed_dict={self.tf_features: next_states})
        target=rewards+0.99*np.amax(allQmax, 1).reshape(allQmax.shape[0], 1)
        targets=actions*target
        batch_size=32
        k = random.randint(0, states.shape[0]-32)
        batch_states=states[k:k+batch_size, :]
        batch_targets=targets[k:k+batch_size, :]
        batch_mask=actions[k:k+batch_size, :]
        self.sess.run(self.train_step, feed_dict={self.tf_features: batch_states, self.Qactions: batch_targets, self.tf_mask: batch_mask})
        print(self.sess.run(self.accuracy, feed_dict={self.tf_features: batch_states, self.Qactions: batch_targets, self.tf_mask: batch_mask}))

def trans_list_np(L):
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
    all_angles=[0.0, 0.78, 1.57, 2.35, 3.14, 3.92, 4.71, 5.50]
    # angle_wind=0.0
    # for x in range(62):
    #     all_angles.append(angle_wind)
    #     angle_wind+=0.1

    trainable=True
    zeta=pi/4
    urmax=pi/4

    boat1=AreaScanBoat(np.array([-47.5,47.5]), psi_w)
    boat2=AreaScanBoat(np.array([47.5,47.5]), psi_w)
    boat3=AreaScanBoat(np.array([-47.5,-47.5]), psi_w)
    boat4=AreaScanBoat(np.array([47.5,-47.5]), psi_w)
    
    if trainable:
        eps=1.0
        M=[boat1.myPointValide, boat2.myPointValide, boat3.myPointValide, boat4.myPointValide]
        opti_step=1
        for i in range(1,5000):
            step=0
            while step<500:
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

                if len(boat1.states)>250000:
                    boat1.states.pop()
                    boat1.rewards.pop()
                    boat1.next_states.pop()
                    boat1.actions.pop()
                    boat2.states.pop()
                    boat2.rewards.pop()
                    boat2.next_states.pop()
                    boat2.actions.pop()
                    boat3.states.pop()
                    boat3.rewards.pop()
                    boat3.next_states.pop()
                    boat3.actions.pop()
                    boat4.states.pop()
                    boat4.rewards.pop()
                    boat4.next_states.pop()
                    boat4.actions.pop()

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

                if step%100==0:
                    angle_wind=random.choice(all_angles)
                    boat1.state[2]=angle_wind
                    boat2.state[2]=angle_wind
                    boat3.state[2]=angle_wind
                    boat4.state[2]=angle_wind
                    #print("reset position")

                if opti_step%1000==0:
                    print("we change the parameter of the copy model")
                    boat1.copy_model_parameter()
                    boat2.copy_model_parameter()
                    boat3.copy_model_parameter()
                    boat4.copy_model_parameter()

                if len(boat1.states)>25000:
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
                    opti_step+=1


            print("new step of the boucle for")
            eps=max(0.1, eps*0.9996)
            boat1.reset()
            boat2.reset()
            boat3.reset()
            boat4.reset()
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

    # boat1.w1.load(, boat1.sess)
    # boat1.b1.load(, boat1.sess)
    # boat1.w2.load(, boat1.sess)

    # boat1.b2.load(, boat1.sess)
    # boat1.ws.load(, boat1.sess)
    # boat1.bs.load(, boat1.sess)
    if not trainable:
        pass
        #main()