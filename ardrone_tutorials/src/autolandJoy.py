#!/usr/bin/env python

# The Joystick Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Joystick Control"
# https://github.com/mikehamer/ardrone_tutorials

# This controller implements the base DroneVideoDisplay class, the DroneController class and subscribes to joystick messages

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials');
import rospy
# Import message types neededcontroller.SetCommand
import std_msgs.msg
import geometry_msgs.msg
import std_srvs.srv
from ardrone_autonomy.msg import Navdata
from rospy.numpy_msg import numpy_msg

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

from dronetools import *

import math
import time

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock
import threading

# Import the joystick message
from sensor_msgs.msg import Joy

# Finally the GUI libraries
#from PySide import QtCore, QtGui
from PyQt4 import QtCore, QtGui

# define the default mapping between joystick buttons and their corresponding actions
ButtonEmergency 		= 8
ButtonLand      		= 1
ButtonTakeoff   		= 0
ButtonToggleAutoLand 	= 3
ButtonToggleAvoidCollision = 5
ButtonCalibrateColor1	= 6
ButtonCalibrateColor2	= 7


# define the default mapping between joystick axes and their corresponding directions
AxisRoll        = 0
AxisPitch       = 1
AxisYaw         = 3
AxisZ           = 4

# define the default scaling to apply to the axis inputs. useful where an axis is inverted
ScaleRoll       = 1.0
ScalePitch      = 1.0
ScaleYaw        = 1.0
ScaleZ          = 1.0

DoAutoLand = False
ALPitch = 0.0
ALRoll = 0.0
ALYaw = 0.0
ALZvel = 0.0

DoAvoidCollision = False


def updateUIText(element, update):
	global ui
	global MainWindow
	global uiLock
	
	noOp = True
	
	uiLock.acquire()
	try:			
		element.setText(QtGui.QApplication.translate("MainWindow", update, None, QtGui.QApplication.UnicodeUTF8))
		time.sleep(.001)
	finally:
		uiLock.release()
	

def updateUIGeo(element, update):
	global ui
	global MainWindow
	global uiLock
	
	noOp = True
	
	#~ uiLock.acquire()
	#~ try:			
		#~ element.setGeometry(update)
		#~ time.sleep(.001)
	#~ finally:
		#~ uiLock.release()
		

# handles the reception of joystick packets
def ReceiveJoystickMessage(data):
	global DoAutoLand
	global DoAvoidCollision
	global hasExecutedLand
	global pitch
	DoAutoLand = data.buttons[ButtonToggleAutoLand]
	DoAvoidCollision = data.buttons[ButtonToggleAvoidCollision]
	#rospy.loginfo("Autoland: %f", DoAutoLand)
	if not DoAutoLand:
		updateUIText(ui.labelAutoLandStatus, "Off")
	
	if data.buttons[ButtonEmergency]==1:
		rospy.loginfo("Emergency Button Pressed")
		controller.SendEmergency()
	elif data.buttons[ButtonLand]==1:
		rospy.loginfo("Land Button Pressed")
		controller.SendLand()
	elif data.buttons[ButtonTakeoff]==1:
		rospy.loginfo("Takeoff Button Pressed")
		controller.SendTakeoff()
		hasExecutedLand = False
	else:
		roll = data.axes[AxisRoll]
		pitch = data.axes[AxisPitch]
		yaw = data.axes[AxisYaw]
		zvel = data.axes[AxisZ]
		
		# Deadzone compensation and remap 0.25-1.0 to 0.0 - 1.0 parabolic
		if abs(roll) < 0.25:
			roll = 0
		else:
			roll = math.copysign(1.382*roll*roll,roll)-0.40*roll+math.copysign(0.018,roll)
			
		if abs(pitch) < 0.25:
			pitch = 0
		else:
			pitch = math.copysign(1.382*pitch*pitch,pitch)-0.40*pitch+math.copysign(0.018,pitch)
			
		if abs(yaw) < 0.25:
			yaw = 0
		else:
			yaw = math.copysign(1.382*yaw*yaw,yaw)-0.40*yaw+math.copysign(0.018,yaw)
			
		if abs(zvel) < 0.25:
			zvel = 0
		else:
			zvel = math.copysign(1.382*zvel*zvel,zvel)-0.40*zvel+math.copysign(0.018,zvel)
			
		
		#rospy.loginfo("roll: %f    pitch: %f    yaw: %f    z: %f", roll*ScaleRoll, pitch*ScalePitch, yaw*ScaleYaw, zvel*ScaleZ)
		#rospy.loginfo(data)
		
		if not DoAutoLand:
			if not DoAvoidCollision or DoAvoidCollision and not collisionDetected:
				setCommand(pitch,roll,yaw,zvel)
	
	if data.buttons[ButtonCalibrateColor1] == 1:
		colorPub.publish(True)
		#	rospy.loginfo("Color change")
	if data.buttons[ButtonCalibrateColor2] == 1:
		colorPub.publish(False)
	
		
def ReceiveLandingPadPosition(position):
	global lpadPosition
	global DoAutoLand
	global hasExecutedLand
	global ui
	global lastReceived
	global vpx
	global vpy
	global landingCondMet
	global t0
	
	#calculate pad velocity
	if not lpadPosition == None:
		vpx = (position.x - lpadPosition.x) / (time.time() - lastReceived) / frameSize[1]*2
		vpy = (position.y - lpadPosition.y) / (time.time() - lastReceived) / frameSize[1]*2
		lastReceived = time.time()
		#rospy.loginfo("vpx: %f   vpy: %f", vpx, vpy)
	
	lpadPosition = position	
	
	# calculate offset from center of frame
	offsetx = (frameSize[0]/2 - position.x)
	offsety = (frameSize[1]/2 - position.y)
	offsetmag = math.hypot(offsetx, offsety)
	
	
	vectorlPadToCenter[0] = offsetx/frameSize[1]*2 # use the same reference size since pitch and roll control should
	vectorlPadToCenter[1] = offsety/frameSize[1]*2 # be the same but the camera is not square
	#rospy.loginfo(vectorlPadToCenter)
	
	updateUIGeo(ui.labelDroneMarker,QtCore.QRect((320 - position.x/2)-15, (180 - position.y/2)-15, 30, 30)) 
	
	calculateAngles(position)
	
	if DoAutoLand and not hasExecutedLand and navdata.altd > 0:
		updateUIText(ui.labelAutoLandStatus, "Centering over pad")
		pitch1 = vectorlPadToCenter[1] - vpy
		roll1 = vectorlPadToCenter[0] - vpx
		pitch2 = vectorlPadToCenter[1] - navdata.vy/1000.0
		roll2 = vectorlPadToCenter[0] - navdata.vx/1000.0
		
		if abs(pitch1) < abs(pitch2):
			pitch = pitch1
		else:
			pitch = pitch2
		if abs(roll1) < abs(roll2):
			roll = roll1
		else:
			roll = roll2
		
		p = abs(pitch)
		r = abs(roll)
		yaw = 0
		zvel = -0.3
		veldrone = math.hypot(navdata.vx, navdata.vy)
		
		if navdata.altd > 800:
			pitch = math.copysign(-0.0240719*p+1.39116*pow(p,2)-0.54625*pow(p,3),pitch)/15.0*navdata.altd/100.0
			roll = math.copysign(-0.0240719*r+1.39116*pow(r,2)-0.54625*pow(r,3),roll)/15.0*navdata.altd/100.0
		else:
			pitch = math.copysign(-0.0240719*p+1.39116*pow(p,2)-0.54625*pow(p,3),pitch)/15.0*navdata.altd/200.0
			roll = math.copysign(-0.0240719*r+1.39116*pow(r,2)-0.54625*pow(r,3),roll)/15.0*navdata.altd/200.0
			
		# Sanity check on control signals
		#~ if abs(pitch) > 5 or abs(roll) > 5:
			#~ rospy.loginfo("pitch: %f, roll: %f, p: %f, r: %f, navdata.altd: %f, vpx %f, vpy %f", pitch, roll, p, r, navdata.altd, vpx, vpy)
			#~ rospy.loginfo("Land")
			#~ hasExecutedLand = True
			#~ controller.SendLand()
			#~ return
		
		if not landingCondMet:
			t0 = time.time()
		acceptableVelDrone = 50/(navdata.altd/1000.0)
		acceptableOffset = 50/(navdata.altd/1000.0)
		if acceptableOffset < 50:
			acceptableOffset = 50
		#~ rospy.loginfo("ALTD: %f    Veldrone: %f    AVD: %f    OffsetMag: %f    AOM: %f", navdata.altd, veldrone, acceptableVelDrone, offsetmag, acceptableOffset)
		updateUIText(ui.labelLandCondVals, "ALTD: %.4f    Veldrone: %.4f    AVD: %.4f    OffsetMag: %.4f    AOM: %.4f" % (navdata.altd, veldrone, acceptableVelDrone, offsetmag, acceptableOffset))
		
		if veldrone < acceptableVelDrone and offsetmag < acceptableOffset:
			rospy.loginfo("Landing conditions met, moving drone down!")
			updateUIText(ui.labelAcceptableVelocity, "Acceptable Velocity: Yes")
			updateUIText(ui.labelAcceptableOffset, "Acceptable Offset: Yes")
			updateUIText(ui.labelLandCond, "Landing Conditions: Met")
			
			landingCondMet = True
			zvel = -0.6 * navdata.altd / 1000 * (navdata.altd > 600)
			setAutoLand(roll, pitch, yaw, zvel)
			if (time.time() - t0) > 1.25*navdata.altd/1000 and navdata.altd < 750:
				rospy.loginfo("Land")
				hasExecutedLand = True
				controller.SendLand()
		else:
			#~ ui.labelLandCond.setText(QtGui.QApplication.translate("MainWindow", "Landing Conditions: Not Met", None, QtGui.QApplication.UnicodeUTF8))
			updateUIText(ui.labelLandCond, "Landing Conditions: Not Met")
			if veldrone > acceptableVelDrone:
				#rospy.loginfo("Unacceptable Velocity")
				updateUIText(ui.labelAcceptableVelocity, "Acceptable Velocity: No")
			else:
				updateUIText(ui.labelAcceptableVelocity, "Acceptable Velocity: Yes")
			if offsetmag > acceptableOffset:
				#rospy.loginfo("Unacceptable Offset")
				updateUIText(ui.labelAcceptableOffset, "Acceptable Offset: No")
			else: 
				updateUIText(ui.labelAcceptableOffset, "Acceptable Offset: Yes")
			landingCondMet = False
			zvel = -0.3 * (navdata.altd > 500)
			#~ rospy.loginfo("!!!!! Automatically moving drone! roll: %f    pitch: %f    yaw: %f    z: %f    vpx: %f    vpy: %f", roll, pitch, yaw, zvel, vpx, vpy)
			setAutoLand(pitch, roll, yaw, zvel)

def setAutoLand(pitch, roll, yaw, zvel):	
	global ALPitch
	global ALRoll
	global ALYaw
	global ALZvel

	ALPitch = pitch
	ALRoll = roll
	ALYaw = yaw
	ALZvel = zvel
	
	
def autoLand():
	global ALPitch
	global ALRoll
	global ALYaw
	global ALZvel
	
	while True:
		#~ rospy.loginfo("AL Thread Loop")
		if DoAutoLand:
			rospy.loginfo("!!!!! Automatically moving drone! roll: %f    pitch: %f    yaw: %f    z: %f", ALPitch, ALRoll, ALYaw, ALZvel)
			setCommand(ALPitch, ALRoll, ALYaw, ALZvel)
			time.sleep(0.25)
			setCommand(0,0,0,0)
			time.sleep(0.15)
			setCommand(0,0,0,ALZvel*1.5)
			time.sleep(0.2)
			if not landingCondMet:
				setCommand(0,0,0,0)
		time.sleep(0.1)

def calculateAngles(position):
	#~ rospy.loginfo("Anyong")
	noOp = True

def setCommand(pitch, roll, yaw, zvel):
	controller.SetCommand(roll, pitch, yaw, zvel)
	updateUIText(ui.labelSetCommand, "pitch: %.4f, roll: %.4f, yaw: %.4f, zvel: %.4f" % (pitch, roll, yaw, zvel))

def ReceiveLandingPadFrameSize(frame):
	global frameSize
	frameSize = frame.data
	#rospy.loginfo(frameSize)

def SwapCamera():
	rospy.loginfo("Swap Camera")
	toggleCam()
	
def TrimDrone():
	rospy.loginfo("Trim Drone")
	trimDrone()

def ReceiveNavdata(nd):
	global navdata
	navdata = nd
	#~ try:
		#~ updateUIText(ui.labelNavDataAltd, "altd: %f" % (navdata.altd))
		#~ updateUIText(ui.labelNavDataVelocity, "vx: %04d vy: %04d vz: %04d" % (navdata.vx, navdata.vy, navdata.vz))
		#~ updateUIText(ui.labelNavDataAccel, "ax: %.4f ay: %.4f az: %.4f" % (navdata.ax, navdata.ay, navdata.az))
		#~ updateUIText(ui.labelNavDataRot, "rotX: %.4f rotY: %.4f rotZ: %.4f" % (navdata.rotX, navdata.rotY, navdata.rotZ))
		#~ updateUIText(ui.labelNavDataBattery, "battery: %f" % (navdata.batteryPercent))
	#~ except:
		#~ rospy.loginfo("UI Update error")
		
	#rospy.loginfo(navdata)
	#rospy.loginfo(nd.altd)
	
def ReceiveCollisionDetected(det):
	global collisionDetected
	collisionDetected = det
	
	if DoAvoidCollision and not collisionDetected:
		# Stop moving drone if had been moving drone
		setCommand(0,0,0,0)
	
def ReceiveAvoidDirection(vec):
	global avoidDirection
	global pitch
	avoidDirection = vec
	
	if DoAvoidCollision and collisionDetected:
		# Move drone in avoidance direction supplied by collision detection routine
		setCommand(pitch,vec.x,0,vec.y)
	

# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_joystick_controller')


	# Next load in the parameters from the launch-file
	ButtonEmergency = int (   rospy.get_param("~ButtonEmergency",ButtonEmergency) )
	ButtonLand      = int (   rospy.get_param("~ButtonLand",ButtonLand) )
	ButtonTakeoff   = int (   rospy.get_param("~ButtonTakeoff",ButtonTakeoff) )
	AxisRoll        = int (   rospy.get_param("~AxisRoll",AxisRoll) )
	AxisPitch       = int (   rospy.get_param("~AxisPitch",AxisPitch) )
	AxisYaw         = int (   rospy.get_param("~AxisYaw",AxisYaw) )
	AxisZ           = int (   rospy.get_param("~AxisZ",AxisZ) )
	ScaleRoll       = float ( rospy.get_param("~ScaleRoll",ScaleRoll) )
	ScalePitch      = float ( rospy.get_param("~ScalePitch",ScalePitch) )
	ScaleYaw        = float ( rospy.get_param("~ScaleYaw",ScaleYaw) )
	ScaleZ          = float ( rospy.get_param("~ScaleZ",ScaleZ) )

	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	display = DroneVideoDisplay()
	controller = BasicDroneController()

	# subscribe to the /joy topic and handle messages of type Joy with the function ReceiveJoystickMessage
	subJoystick = rospy.Subscriber('/joy', Joy, ReceiveJoystickMessage)
	pitch = 0
	
    # start autoLand thread
	alThread = threading.Thread(target=autoLand)
	alThread.setDaemon(True)
	alThread.start()

	# variables to hold landing pad data
	lpadPosition = None
	frameSize = None
	vectorlPadToCenter = [0,0]
	hasExecutedLand = False
	vpx = 0
	vpy = 0
	lastReceived = time.time()
	landingCondMet = False

	# display camera view
	display.show()
	
	# display main window
	ui = Ui_MainWindow()
	MainWindow = QtGui.QMainWindow()
	ui.setupUi(MainWindow)
	MainWindow.show()
	QtCore.QObject.connect(ui.pushButtonSwapCamera, QtCore.SIGNAL("clicked()"), SwapCamera)
	QtCore.QObject.connect(ui.pushButtonTrim, QtCore.SIGNAL("clicked()"), TrimDrone)
	uiLock = Lock()
	
	# subscribe to landing pad tracker
	sublpadTrackerFrameSize = rospy.Subscriber('/landingPadTracker/frameSize', numpy_msg(std_msgs.msg.Int32MultiArray), ReceiveLandingPadFrameSize)
	sublpadTrackerPosition = rospy.Subscriber('/landingPadTracker/landingPadPosition', numpy_msg(geometry_msgs.msg.Point), ReceiveLandingPadPosition)

	# Color calibration publisher
	colorPub = rospy.Publisher('landingPadTracker/setColorMsg',std_msgs.msg.Bool)

	# connect to ardrone service
	toggleCam = rospy.ServiceProxy('ardrone/togglecam', std_srvs.srv.Empty)
	trimDrone = rospy.ServiceProxy('ardrone/flattrim', std_srvs.srv.Empty)
	
	# subscribe to collision program
	subCollisionDetected = rospy.Subscriber('collisionDetect/willCollide', numpy_msg(std_msgs.msg.Bool), ReceiveCollisionDetected)
	subAvoidDirection = rospy.Subscriber('collisionAvoid/avoidanceDirection', numpy_msg(geometry_msgs.msg.Point), ReceiveAvoidDirection)
	collisionDetected = False
	avoidDirection = None
    
	
	# subscribe to navdata
	navdata = None
	sublpadTrackerPosition = rospy.Subscriber('/ardrone/navdata', numpy_msg(Navdata), ReceiveNavdata)
	
	# execute QT application
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
