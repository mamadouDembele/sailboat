#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import time
import math
import struct
import sys

import threading
import socket

import numpy as np
import robotics_toolbox_v0_1_py3 as rb

import signal


# reset variables 
vAngleCmdJoint1=0.0
vAngleCmdJoint2=0.0
vAngleCmdJoint3=0.0
vAngleValJoint1=0.0
vAngleValJoint2=0.0
vAngleValJoint3=0.0
gripperCmd=0.0
gripperLeftX=0.0
gripperLeftY=0.0
gripperLeftZ=0.0
gripperRightX=0.0
gripperRightY=0.0
gripperRightZ=0.0

# initiate communication thread with V-Rep
simulation_alive = True
simulation_end = False

    
def clean_end():
    global simulation_alive,simulation_end
    print ("Start ending simulation ...")
    simulation_end = True
    while simulation_alive:
        time.sleep(0.1)
    print ("End ...")
    sys.exit(0)

def interrupt_clean_end(signal, frame):
    clean_end()

signal.signal(signal.SIGINT, interrupt_clean_end)


# thread to xchange data with vrep
#  - reads the sensors values
#  - set the motors  
def vrep_com_socket(s,ev):
    print ("enter vrep sock com ",s,ev) 
    global vAngleCmdJoint1,vAngleCmdJoint2,vAngleCmdJoint3
    global vAngleValJoint1,vAngleValJoint2,vAngleValJoint3
    global gripperCmd,gripperLeftX,gripperLeftY,gripperLeftZ,simuTime
    global gripperRightX,gripperRightY,gripperRightZ
    global simulation_alive,simulation_end
    #print (s)
    #print (ev)
    RECV_BUFFER = 4096 # buffer size 
    while simulation_alive:
        #wait to accept a connection - blocking call
        conn, addr =  s.accept()
        print ('Connected with ' + addr[0] + ':' + str(addr[1]))
        #print (conn)

        while simulation_alive:
            #print ("socket read",conn)
            data = conn.recv(RECV_BUFFER)
            #print ("len(data)",len(data))
            hd0,hd1,sz,lft,vAngleValJoint1,vAngleValJoint2,vAngleValJoint3,gripperLeftX,gripperLeftY,gripperLeftZ,gripperRightX,gripperRightY,gripperRightZ,simulationTime = struct.unpack('<ccHHffffffffff',data)
            #print (vAngleValJoint1,vAngleValJoint2,vAngleValJoint3)
            #print (vAngleCmdJoint1,vAngleCmdJoint2,vAngleCmdJoint3,float(gripperCmd))
            strSend = struct.pack('<BBHHffff',data[0],data[1],16,0,vAngleCmdJoint1,vAngleCmdJoint2,vAngleCmdJoint3,float(gripperCmd))
            #print (vAngleCmdJoint1,vAngleCmdJoint2,vAngleCmdJoint3,float(gripperCmd))
            conn.send(strSend)
            time.sleep(0.010)
            if simulation_alive:
                ev.set()  # start new timer if robot is alive
            else:
                print ("Out of V-Rep Socket ...")
                s.shutdown(socket.SHUT_RD)
                break 

            # end of simulation
            if simulation_end:
                simulation_alive = False
                print ("Out of V-Rep Connection ...")
                #break

vrobot_ready = threading.Event()
vrobot_ready.clear()

# socket connection to V-REP
HOST = 'localhost'  # IP of the sockect
PORT = 25010 # port (set similarly in v-rep)

sck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print ('Socket created')

# bind socket to local host and port
try:
    # prevent to wait for  timeout for reusing the socket after crash
    # from :  http://stackoverflow.com/questions/29217502/socket-error-address-already-in-use
    sck.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sck.bind((HOST, PORT))
except sck.error as msg:
    print (msg)
    #sys.exit()
print ('Socket bind completed')
             
# start listening on socket
sck.listen(10)
print ('Socket now listening')


#sk = sck
ev = vrobot_ready
vrep = threading.Thread(target=vrep_com_socket,args=(sck,ev,))
vrep.start()

# wait for vrobot to be ready
vrobot_ready.wait()
print ("Robot ready ...")



def setJointAngles(v,sleepTime):
    global vAngleCmdJoint1,vAngleCmdJoint2,vAngleCmdJoint3
    vAngleCmdJoint1 = v[0]
    vAngleCmdJoint2 = v[1]
    vAngleCmdJoint3 = v[2]
    time.sleep(sleepTime)
    
def setGripper(cmd,sleepTime):
    global gripperCmd
    gripperCmd = cmd
    time.sleep(sleepTime)


def getJointAngles():
    global vAngleValJoint1,vAngleValJoint2,vAngleValJoint3
    return np.asarray([vAngleValJoint1,vAngleValJoint2,vAngleValJoint3])

def getGripperTouchLocations():
    global gripperLeftX,gripperLeftY,gripperLeftZ
    global gripperRightX,gripperRightY,gripperRightZ
    gripperTouchLocations = np.zeros((2,3))
    gripperTouchLocations[0,:] = [gripperLeftX,gripperLeftY,gripperLeftZ]
    gripperTouchLocations[1,:] = [gripperRightX,gripperRightY,gripperRightZ]
    return gripperTouchLocations

def getGripperCenter():
    v = getGripperTouchLocations()
    vc = (v[0,:]+v[1,:])/2.0
    return vc

if __name__ == "__main__":
    LA = 0.53
    LB = 0.6
    LC = 0.38
    LD = 0.48

    lk1 = rb.Link(A=LB,alpha=-np.pi/2.0,D=LA)
    lk2 = rb.Link(A=LC,alpha=0.0,D=0.0)
    lk3 = rb.Link(A=LD,alpha=np.pi/2.0,D=0.0)
    links = [lk1, lk2, lk3]
    arm=rb.Robot(links,name="test arm")

    setGripper(0.0,1.0)
    qzero=np.asarray([0.,0.,0.])
    qi=qzero
    setJointAngles(qi,0.0)
    P=np.asarray([0., 0., 0.])
    Ph=rb.e2h(P)

    xmv=0.0
    ymv=0.8
    zmv=0.05

    xmr=-0.8
    ymr=0.8
    zmr=0.05

    xmj=0.8
    ymj=0.8
    zmj=0.05
    cmd=0
    Jaune=[[xmj, ymj, 0.53, 0],
    [xmj-0.2, ymj-0.2, 0.3, 0],
    [xmj-0.1, ymj-0.1, zmj, 0],
    [xmj+0.04, ymj+0.04, zmj+0.03, 0],
    [1],
    [xmj+0.05, ymj+0.05, 0.45, 1],
    [xmv, ymv+0.05, 0.3, 1],
    [xmv, ymv, 0.3,1],
    [xmv, ymv+0.03, 0.15,1],
    [0],
    [xmv, ymv-0.01, 0.3,0],
    [-0.5,ymv+0.5, 0.3, 0]
    ]
    print("     Recuperation du cylindre jaune      ")
    print("=========================================")
    k=1
    ts=10.0
    for M in Jaune:
        print("etape ", k)
        if len(M)>1:
            if k==4:
                ts=15.0
            if k==12:
                ts=7.0
            qi,nm,count = rb.ikine(arm,rb.transl(M[0:3]),qi,[1, 1, 1, 0, 0, 0])
            qi = np.squeeze(np.asarray(qi))
            Mi = rb.h2e(np.transpose(rb.fkine(arm,qi).dot(Ph)))
            print ("M (target) = ",M[0:3])
            print ("M (Check)  = ",Mi)    
            setJointAngles(qi,ts)
            setGripper(M[3],1.0)
        else:
            setGripper(M[0],10.0)
        k+=1


    Rouge=[[xmr+0.2, ymr-0.2, 0.3, 0],
    [xmr+0.05, ymr-0.05, zmr, 0],
    [xmr-0.04, ymr+0.04, zmr+0.015, 0],
    [1],
    [xmr-0.05, ymr+0.05, 0.45, 1],
    [-0.8, 1.5, 0.53, 1],
    [xmv+0.07, ymv+0.05, 0.3, 1],
    [xmv+0.07, ymv+0.03, 0.3, 1],
    [xmv, ymv, 0.3,1],
    #[xmv, ymv-0.03, 0.3,1],
    [0],
    [xmv, ymv, 0.33,0],
    [0.5, 1.5, 0.6,0],
    ]
    print("     Recuperation du cylindre rouge      ")
    print("=========================================")
    ts=10.0
    for M in Rouge:
        print("etape ", k)
        if len(M)>1:
            if k==3:
                ts=15.0
            qi,nm,count = rb.ikine(arm,rb.transl(M[0:3]),qi,[1, 1, 1, 0, 0, 0])
            qi = np.squeeze(np.asarray(qi))
            Mi = rb.h2e(np.transpose(rb.fkine(arm,qi).dot(Ph)))
            print ("M (target) = ",M[0:3])
            print ("M (Check)  = ",Mi)    
            setJointAngles(qi,0.1)
            setGripper(M[3],ts)
        else:
            setGripper(M[0],10.0)
        k+=1


    

    #     print("Attrape")
    #     break
    #     # ang = getJointAngles()
    #     # print ("Set Angles     = ",np.round(qi*180.0/np.pi,1))
    #     # print ("Get Angles     = ",np.round(ang*180.0/np.pi,1))
    #     # print ("get gripper center :",getGripperCenter())

    clean_end()
