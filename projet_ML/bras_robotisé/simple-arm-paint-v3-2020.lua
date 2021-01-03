-- Following function writes data to the socket (only single packet data for simplicity sake):
writeSocketData=function(client,data)
    local header=string.char(59,57,math.mod(#data,256),math.floor(#data/256),0,0)
    -- Packet header is (in this case): headerID (59,57), dataSize (WORD), packetsLeft (WORD) but not used here
    client:send(header..data)
end

-- Following function reads data from the socket (only single packet data for simplicity sake):
readSocketData=function(client)
    -- Packet header is: headerID (59,57), dataSize (WORD), packetsLeft (WORD) but not used here
    local header=client:receive(6)
    if (header==nil) then
        return(nil) -- error
    end
    if (header:byte(1)==59)and(header:byte(2)==57) then
        local l=header:byte(3)+header:byte(4)*256
        return(client:receive(l))
    else
        return(nil) -- error
    end
end

-- Use sleeping function of socket library
function sleep(sec)
    socket.select(nil,nil,sec)
end 

-- Gripper open/close
function gripper_actuation_open()
    local v=-gripperMotorVelocity
    local data=sim.getIntegerSignal('RG2_open')
    if data and data~=0 then
        v=gripperMotorVelocity
    end
    sim.setJointForce(gripperMotorHandle,gripperMotorForce)
    sim.setJointTargetVelocity(gripperMotorHandle,v)
end
function gripper_actuation_close()
    local v=gripperMotorVelocity
    local data=sim.getIntegerSignal('RG2_close')
    if data and data~=0 then
        v=-gripperMotorVelocity
    end
    sim.setJointForce(gripperMotorHandle,gripperMotorForce)
    sim.setJointTargetVelocity(gripperMotorHandle,v)
end

--- convert and round angle for display
function convertAngles(v)
   local vr,decim
   decim = 100.0
   vr = v*180.0/math.pi
   vr = math.floor(vr*decim+0.5)/decim
   return vr
end
   
threadFunction=function()
    while simGetSimulationState()~=sim_simulation_advancing_abouttostop do
        -- The executable could be launched. Now build a socket and connect to the server:
        --simAddStatusbarMessage("try opening socket ...")
        local socket=require("socket")
        local client=socket.tcp()
        simSetThreadIsFree(true) -- To avoid a bief moment where the simulator would appear frozen
        local result=client:connect('127.0.0.1',portNb)
        simSetThreadIsFree(false)
        if (result==1) then
           -- We could connect to the server
            while (simGetSimulationState()~=sim_simulation_advancing_abouttostop) do
                --local clock = os.clock
                --local t0 = clock()
                -- Send the 4 sonar sensors, the 2 encoders and the simulation time 
                -- to the vdart server application:
                -- Prepare the data to send:
                local dataOut={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,simGetSimulationTime()}
                angleValJoint1 = simGetJointPosition(joint1)
                angleValJoint2 = simGetJointPosition(joint2)
                angleValJoint3 = simGetJointPosition(joint3)
		gripperPrm1 = 0.0
                dataOut[1]=angleValJoint1
                dataOut[2]=angleValJoint2
                dataOut[3]=angleValJoint3
                gripperLeftTouchLocation = simGetObjectPosition(gripperLeftTouch,-1)
                gripperRightTouchLocation = simGetObjectPosition(gripperRightTouch,-1)
                for i=1,3,1 do
                    dataOut[i+3]=gripperLeftTouchLocation[i]
                    dataOut[i+6]=gripperRightTouchLocation[i]
                end
                --print (dataOut[4],dataOut[5],dataOut[6])
                
                -- Pack the data as a string:
                dataPacked=simPackFloats(dataOut)
                -- Send the data:getDrawingObject=function(objectHandle)

                writeSocketData(client,dataPacked)
                -- Read the reply from the server:
                local returnData=readSocketData(client)
                if (returnData==nil) then
                    break -- Read error
                end
                -- unpack the received data:
                targetCmd=simUnpackFloats(returnData)
                --print (targetCmd[1],targetCmd[2],targetCmd[3])
                simSetJointTargetPosition(joint1,targetCmd[1])
                simSetJointTargetPosition(joint2,targetCmd[2])
                simSetJointTargetPosition(joint3,targetCmd[3])
                gripperCmd = targetCmd[4]
                if gripperCmd == 1.0 then
		   gripper_actuation_open()
                else
		   gripper_actuation_close()
                end

                if cnt == cntDispl then
                    cnt = 0
                    local stOut = "Angles J1:"..convertAngles(angleValJoint1)..", J2:"..convertAngles(angleValJoint2)..", J3:"..convertAngles(angleValJoint3)..", Gripper:"..gripperCmd
                    print (stOut)
                    simAddStatusbarMessage(stOut)
                end
                cnt = cnt+1

                 -- Now don't waste time in this loop if the simulation time hasn't changed! This also synchronizes this thread with the main script
                simSwitchThread() -- This thread will resume just before the main script is called again
            end
        else
            --simAddStatusbarMessage("socket not open ...")
        end  
 
        client:close()
    end
end

-- Initialization:
simSetThreadSwitchTiming(50) -- We wanna manually switch for synchronization purpose (and also not to waste processing time!)

angleCmdJoint1=0.0
angleCmdJoint2=0.0
angleCmdJoint3=0.0

-- get handles of sonar sensor
joint1 = simGetObjectHandle("Joint1")
joint2 = simGetObjectHandle("Joint2")
joint3 = simGetObjectHandle("Joint3")

angleValJoint1 = simGetJointPosition(joint1)
angleValJoint2 = simGetJointPosition(joint2)
angleValJoint3 = simGetJointPosition(joint3)

simSetJointTargetPosition(joint1,angleCmdJoint1)
simSetJointTargetPosition(joint2,angleCmdJoint2)
simSetJointTargetPosition(joint3,angleCmdJoint3)

--jetHandle=simGetObjectHandle("PaintNozzleJet")
--joint0Handle=simGetObjectHandle("PaintNozzleJoint0")
--joint1Handle=simGetObjectHandle("PaintNozzleJoint1")
--jetAngle=30*math.pi/180
--density=16 --64
--objList={}
--allDrawingObjects={}
--mode=sim_drawing_trianglepoints
--colorIndex=2
--itemSize=3
--bufferSize=30000
--paintingEnabled=true

--tool = simGetObjectHandle("Tool")
gripperConnector=sim.getObjectHandle('RG2_attachPoint')
gripperObjectSensor=sim.getObjectHandle('RG2_attachProxSensor')
gripperMotorHandle=sim.getObjectHandle('RG2_openCloseJoint')
gripperMotorVelocity=0.05 -- m/s
gripperMotorForce=20 -- N
gripperRightTouch=sim.getObjectHandle('RG2_rightTouch')
gripperLeftTouch=sim.getObjectHandle('RG2_leftTouch')



cnt=0
cntDispl=1
portNb = 25010
-- Start Simple Robotic Arm 
simAddStatusbarMessage('start: simple robotic arm at port '..portNb)
-- Execute the thread function:
res=false
err=" not launched delibarately "
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
   simAddStatusbarMessage('Lua runtime error: '..err)
end

-- Clean-up:

