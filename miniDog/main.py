import numpy as np
#import matplotlib.pyplot as plt
#import mpl_toolkits.mplot3d.axes3d as p3
#import matplotlib.animation as animation
from trajectoryInterpolation import *		#import all class and functions
from dogPlatform import *
import threading
from gaitGenerator import *
from HXServo import *

from time import sleep
#import Adafruit_PCA9685
import serial

frameNum = 6 
stepTime = 0.05		#units:s
alfa = 0.5		#smooth coefficiency
dog = dogPlatform()
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout = 0.005)
servo = HXServo(ser)
#pwm = Adafruit_PCA9685.PCA9685()
#pwm.set_pwm_freq(50)
	
def set_servo_angle(angle):
#	for i in range(0, 8):
#		pwm.set_pwm(i, i, int(angle[i]*2.276+102.4))
	servo.setPosition(2, int(angle[0] * 11.378))
	servo.setPosition(6, int(angle[1] * 11.378))
	servo.setPosition(3, int(angle[2] * 11.378))
	servo.setPosition(7, int(angle[3] * 11.378))
	servo.setPosition(4, int(angle[4] * 11.378))
	servo.setPosition(8, int(angle[5] * 11.378))
	servo.setPosition(5, int(angle[6] * 11.378))
	servo.setPosition(9, int(angle[7] * 11.378))
def motorDriver(joints):
	print 'joints:', joints
	global bias
	leg1 = [-joints[0][1] + bias[0][0],   joints[0][2] + bias[0][1]]
	leg2 = [ joints[1][1] + bias[1][0],  -joints[1][2] + bias[1][1]]
	leg3 = [-joints[2][1] + bias[2][0],  -joints[2][2] + bias[2][1]]
	leg4 = [ joints[3][1] + bias[3][0],   joints[3][2] + bias[3][1]]
	leg = leg1 + leg2 + leg3 + leg4
	set_servo_angle(leg)	

def myMain():
	global bias
	bias = np.array([[-0, 0, -0],\
			 [-0, -0, 0],\
			 [-0, -0, -0],\
			 [-0, 0, 0]])
	print 'Setting servo mode'
	for i in range(2, 10):
		print i
		servo.setWorkingMode(i,2)
	print 'Getting bias parameters...'
	if False:
		bias[0,0] = servo.getPosition(2)*0.08789
		bias[0,1] = servo.getPosition(6)*0.08789
		bias[1,0] = servo.getPosition(3)*0.08789
		bias[1,1] = servo.getPosition(7)*0.08789
		bias[2,0] = servo.getPosition(4)*0.08789
		bias[2,1] = servo.getPosition(8)*0.08789
		bias[3,0] = servo.getPosition(5)*0.08789
		bias[3,1] = servo.getPosition(9)*0.08789
	else:
		bias[0,0] = 154
		bias[0,1] = 194
		bias[1,0] = 209
		bias[1,1] = 195
		bias[2,0] = 153
		bias[2,1] = 172
		bias[3,0] = 213
		bias[3,1] = 188
	print "bias:", bias
	
	initLegPose = np.array([ [0, 0, 0],\
				 [0, 0, 0],\
				 [0, 0, 0],\
				 [0, 0, 0]])
	motorDriver(initLegPose)
	
	#init plot
	
	pace = gaitPace(-90)
	trot = gaitTrot(-90)
	walk = gaitWalk(-100)
	Tcps = walk.getTcps(10, 30, frameNum)
	print 'Tcps:', Tcps[1]
	dTcps = walk.getTcpsRelative(10, 30, frameNum)
	print 'length of Tcps:', len(Tcps)
	print 'dTcps:', dTcps[0]
	dog.setLegTcp(Tcps[0])
	legPose = dog.getLegPose()
	print 'legPose:', legPose
	print("Finish init")
	num = 0
	print "MaxTemperature:", servo.getMaxTemperature(2)
	print 'Tcps:\n', Tcps[-1]
	print 'dTcps:\n', dTcps[-1]
	while False:
		legPose[0][1] = 0
		legPose[0][2] = 50
		legPose[1][1] = 0
		legPose[1][2] = 0
		legPose[2][1] = 0
		legPose[2][2] = 0
		legPose[3][1] = 0
		legPose[3][2] = 50
		motorDriver(legPose)
		sleep(0.4)
		legPose[0][1] = 0
		legPose[0][2] = 0
		legPose[1][1] = 0
		legPose[1][2] = 50
		legPose[2][1] = 0
		legPose[2][2] = 50
		legPose[3][1] = 0
		legPose[3][2] = 0
		motorDriver(legPose)	
		sleep(0.4)
	while False:
		print "num:", num
		if num  == 0:
			tcp = alfa * (dog.getLegTcp() + dTcps[num]) + (1 - alfa) * Tcps[num]
		else:
			tcp = alfa * (dog.getLegTcp() + dTcps[num] - dTcps[num - 1]) + (1 - alfa) * Tcps[num]
		dog.setLegTcp(tcp)
		legPose = dog.getLegPose()
		motorDriver(legPose)
#		if num == 2 * frameNum - 1:
#			break
		num = (num + 1) % (2 * frameNum)
		sleep(stepTime)
if __name__== '__main__':
	myMain()
