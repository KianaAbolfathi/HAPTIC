# Import necessary libraries
from ctypes import cast
import threading
import DobotDllType as dType
import os
import time
import copy
import math
import csv
from datetime import datetime
GREEN = "\033[32m"
RED = "\033[31m"
RESET = "\033[0m"  # Reset color to default

vel = 10
# Check if the named pipe exists, and if not, create it
pipe_name = 'mypipe'
if not os.path.exists(pipe_name):
    os.mkfifo(pipe_name)
os.system('cls')
print("Reading data from C++",end='\n')

# Define a dictionary for connection status messages
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

# Load Dll and get the CDLL object
api = dType.load()

# Connect Dobot
state = dType.ConnectDobot(api, "COM4", 115200)[0]
print("Connect status:",CON_STR[state])
print("Please wait! Calibrating and Homing......")  
time.sleep(1)

# create a CSV file to stare performance data
outputFile = open('PerformanceTestOP.csv', mode='w', newline='')
csv_writer = csv.writer(outputFile)
# Write the headder into CSV file
csv_writer.writerow(["INPUT X","INPUT Y","INPUT Z","INPUT B","   ","OUTPUT X","OUTPUT Y","OUTPUT Z","OUTPUT B","OUTPUT VEL"])

if (state == dType.DobotConnect.DobotConnect_NoError):
    
    # Clean Command Queued
    dType.SetQueuedCmdClear(api)
    
    # Set Motion Params Setting
    dType.SetHOMEParams(api, 200, 200, 200, 200, isQueued = 1)
    
    # turn on the SW port on for 12v output as power of the motor.
    dType.SetEndEffectorSuctionCupEx(api, 1, 1)

    # Perform  Home
    dType.SetHOMECmd(api, temp = 1, isQueued = 0)
        
    # Start to Execute Command Queue
    dType.SetQueuedCmdStartExec(api)
    
    # Lock into the home coordinates initially 
    lock=1
    lockS="LOCKED"
    print(" ")
    print("         NOVINT FALCON DATA                                                                  DOBOT MAGICIAN DATA")
    print(" ")
    while True:
        #dType.SetPTPCommonParams(api, 200, 200, isQueued = 0)
        dType.SetJOGCoordinateParams(api, vel, 200, vel, 200, vel, 200, 40, 10,  isQueued=0)

        # read values from the named pipes
        valuex = 0
        valuey = 0
        valuez = 0
        valueb = 0

        # assign the read values from the pipes to variables.
        pipe = open(pipe_name, 'r')
        input_string = pipe.readline().rstrip()
        numbers = input_string.split()
        if (numbers):
            input_valuex = float(numbers[0])
            input_valuey = float(numbers[1])
            input_valuez = float(numbers[2])
            input_valueb = float(numbers[3])

            # process the input values from the pipes
            if(input_valuex):
                valuex = ((float(input_valuex))*(float(input_valuex)/20)**2)/2
            if(input_valuey):
                valuey = ((float(input_valuey))*(float(input_valuey)/20)**2)/2
            if(input_valuez):
                valuez = ((float(input_valuez))*(float(input_valuez)/20)**2)/2
            if(input_valueb):
                valueb = float(input_valueb)
                
        timestamp = time.time()
        milliseconds = int((timestamp - int(timestamp)) * 1000)  # Extract milliseconds

        current_time = datetime.fromtimestamp(timestamp)
        formatted_time = current_time.strftime("%Y-%m-%d %H:%M:%S") + f".{milliseconds:03d}"

        # Write the output data into CSV file
        csv_writer.writerow([input_valuex, input_valuey, input_valuez, input_valueb,"",valuex, valuey, valuez, valueb, vel])

        # variable to set the command to move along which coordinate - (1 and 2 along X-axis, 3 and 4 along Y-axis, 5 and 6 along Z-axis)
        cmd = 0 
        CatMani = "PAUSE"
        # rotate the catheter driver if 8 or 2 is read from the buttons
        if valueb == 8:
            cmd = 7
        if valueb == 2:
            cmd = 8
        if valueb == 0:
            dType.SetIODO(api,13,0,1)
            dType.SetIODO(api,10,0,1)
            CatMani = "PAUSE"
        
        # drive the catheter back or forth if 1 or 4 is read from the buttons    
        if valueb == 1:
            dType.SetIODO(api,13,1,1)
            dType.SetIODO(api,10,0,1)
            CatMani = "IN"
        if valueb == 4:
            dType.SetIODO(api,13,0,1)
            dType.SetIODO(api,10,1,1)
            CatMani = "OUT"

        # lock the coordinate values of the robotic arm so that it wont move while driving the catheter in or out 
        if valueb == 5:
            lock = 0
            lockS = "UNLOCKED"
        if valueb == 10:
            lock = 1
            lockS = "LOCKED"
            

        # Display the output
        #print("POS- X-Axis: " , math.trunc(valuex), " Y-Axis: ", math.trunc(valuey), " Z-Axis: ", math.trunc(valuez)," | BTN- ", input_valueb," cordinate lock- ", lock,"          ")
        #print(" ")
        #print("Current Coordinate - X: " , dType.GetPose(api)[0], " Y-Axis: ", dType.GetPose(api)[1], " Z-Axis: ", dType.GetPose(api)[2], " Rotation: ", dType.GetPose(api)[3]," | Catheter-Manipulator- ", CatMani, end='\r')
        
        print("| Falcon- X: "+ RED,math.trunc(valuex),RESET + " Y: "+ RED, math.trunc(valuey),RESET + " Z: "+ RED, math.trunc(valuez),RESET +" BTN: "+ RED, math.trunc(input_valueb),RESET +" |          | Dobot coordinates- lock: "+ GREEN, lockS,RESET +" X: "+ GREEN,round(dType.GetPose(api)[0],1), RESET +" Y: "+ GREEN, round(dType.GetPose(api)[1],1), RESET +" Z: "+ GREEN, round(dType.GetPose(api)[2],1), RESET +" R: "+ GREEN, round(dType.GetPose(api)[3],1),RESET +" Catheter Manipulator: "+ GREEN, CatMani,RESET +" |      ", end='\r')
        
        
        # set coordinates for the dobot reading the valeus from the pipe.
        if lock == 0:
            if valuex>0:
               cmd = 1                
            elif valuex<-0:
               cmd = 2
             #When including Y and Z direction, move this else statement to the bottom, after cmd=8
            #The following can be commented out to avoid movement in Y and Z direction. 
            elif valuey>0:
               cmd = 3 
            elif valuey<-0:
               cmd = 4 
            elif valuez>0:
               cmd = 5 
            elif valuez<-0:
               cmd = 6
            elif valueb == 8:
               cmd = 7
            elif valueb == 2:
               cmd = 8
            else:
                cmd = 0 
        #Making a scaled down version of the value x for Magnetic Guidewire's experiment
        Guidewire_valuex = valuex/10
        Guidewire_valuey = valuey/10
        Guidewire_valuez = valuez/10
        #map the valocity of the arm according to the radius of the haptic devices input
        vel = math.sqrt(pow(Guidewire_valuex,2)+pow(Guidewire_valuey,2)+pow(Guidewire_valuez,2))
        
        # send the command to move the dobot
        dType.SetJOGCmd(api, 0, cmd, 0)

        # clear the alarms to avoid pause while on execution of tasks.
        dType.ClearAllAlarmsState(api)

        #close all pipes
        pipe.close()
        
# Stop to Execute Command Queued
dType.SetQueuedCmdStopExec(api)
    
# Disconnect Dobot
dType.DisconnectDobot(api)
