# this is a class that controls four legged robot
#import matplotlib as mpl
#from mpl_toolkits.mplot3d import Axes3D
import numpy as np
#import matplotlib.pyplot as plt

class dogPlatform:
	'''Record the dog's status'''
	def __init__(self):
		'''Initialize the dog's status'''
		self.bodyPose = np.zeros(6)		#record dog body's pose (x,y,z,yaw,pitch,roll)
		self.legTcp = np.zeros((4,4))		#record four legPose(x,y,z,1)
		self.legPose = np.zeros((4,3))		#record four legs' joins
		#dimensiton parameters
		self.width = 100
		self.height = 165
		#leg order: front_left, front_right, rear_left, rear_right
		self.legFix = np.array([[-self.width / 2., self.height / 2., 0, 1],\
					[self.width / 2., self.height / 2., 0, 1],\
					[-self.width / 2., -self.height / 2., 0, 1],\
					[self.width / 2., -self.height / 2., 0, 1]])
		self.legUpper = np.array([[0, 0, -90],\
					  [0, 0, -90],\
					  [0, 0, -90],\
					  [0, 0, -90]])
		self.legLow = np.array([[0, 0, -75],\
					[0, 0, -75],\
					[0, 0, -75],\
					[0, 0, -75]])
		dogPlatform.forwardKinematics(self)
	def inverseKinematics(self):
		'''Calculate leg joints from Tcp'''
		for i in range(0,4):
			dis = self.legFix[i] - self.legTcp[i]
			legLength = np.sqrt(dis[0]**2 + dis[1]**2 + dis[2]**2)
			joint_0 = np.degrees(-np.arcsin(dis[1] / legLength))
			joint_1 = np.degrees((110 - legLength) / 22.5)
			self.legPose[i][0] = 0
			self.legPose[i][1] = joint_0
			self.legPose[i][2] = joint_1

	def forwardKinematics(self):
		'''Calculate leg Tcp from joints'''
                for i in range(0,4):
			a = np.radians(self.legPose[i][0])
	                b = np.radians(self.legPose[i][1])
	                c = np.radians(self.legPose[i][2])
			transFix = np.matrix([[1, 0, 0, self.legFix[i][0]],\
					      [0, 1, 0, self.legFix[i][1]],\
					      [0, 0, 1, self.legFix[i][2]],\
					      [0, 0, 0, 1]]) 
			rotate1 = np.matrix([[np.cos(a), 0, np.sin(a), 0],\
	                                     [0, 1, 0, 0],\
	                                     [-np.sin(a), 0, np.cos(a), 0],\
					     [0, 0, 0, 1]])
	                rotate2 = np.matrix([[1, 0, 0, 0],\
	                                     [0, np.cos(b), -np.sin(b), 0],\
	                                     [0, np.sin(b), np.cos(b), 0],\
				             [0, 0, 0, 1]]) 
	
	                transUpper = np.matrix([[1, 0, 0, self.legUpper[i][0]],\
						[0, 1, 0, self.legUpper[i][1]],\
						[0, 0, 1, self.legUpper[i][2]],\
						[0, 0, 0, 1]]) 
	                rotate3 = np.matrix([[1, 0, 0, 0],\
	                                     [0, np.cos(c), -np.sin(c), 0],\
	                                     [0, np.sin(c), np.cos(c), 0],\
					     [0, 0, 0, 1]])
					
			transLow = np.matrix([[1, 0, 0, self.legLow[i][0]],\
					      [0, 1, 0, self.legLow[i][1]],\
					      [0, 0, 1, self.legLow[i][2]],\
					      [0, 0, 0, 1]]) 
			transfer = transFix * rotate1 * rotate2 * transUpper * rotate3 * transLow
			p = np.array([0, 0, 0, 1])
                	self.legTcp[i] = np.dot(transfer,p)

	def setLegTcp(self, legTcp):
		'''Set dog's four legs' Tcp position'''
		self.legTcp = legTcp
		dogPlatform.inverseKinematics(self)			#update dog's four legs' joints
	
	def setLegPose(self, legPose):
		'''Set dog's four legs' joints pose'''
		self.legPose = legPose
		dogPlatform.forwardKinematics(self)

	def getLegPose(self):
		'''Get dog's four legs' joints pose'''
		return self.legPose

	def getLegTcp(self):
		'''Get dog's four legs' Tcp position'''
		return self.legTcp

	def getLegKnee(self):
		'''Get dog's four legs' knee position'''
		knee = np.zeros((4,4))
                for i in range(0,4):
                        a = np.radians(self.legPose[i][0])
                        b = np.radians(self.legPose[i][1])
                        transFix = np.matrix([[1, 0, 0, self.legFix[i][0]],\
                                              [0, 1, 0, self.legFix[i][1]],\
                                              [0, 0, 1, self.legFix[i][2]],\
                                              [0, 0, 0, 1]])
                        rotate1 = np.matrix([[np.cos(a), 0, np.sin(a), 0],\
                                             [0, 1, 0, 0],\
                                             [-np.sin(a), 0, np.cos(a), 0],\
                                             [0, 0, 0, 1]])
                        rotate2 = np.matrix([[1, 0, 0, 0],\
                                             [0, np.cos(b), -np.sin(b), 0],\
                                             [0, np.sin(b), np.cos(b), 0],\
                                             [0, 0, 0, 1]])

                        transUpper = np.matrix([[1, 0, 0, self.legUpper[i][0]],\
                                                [0, 1, 0, self.legUpper[i][1]],\
                                                [0, 0, 1, self.legUpper[i][2]],\
                                                [0, 0, 0, 1]])
                        transfer = transFix * rotate1 * rotate2 * transUpper
                        p = np.array([0, 0, 0, 1])
                        knee[i] = np.dot(transfer,p)
		return knee

	def getFixPoints(self):
		return self.legFix

def dogShow(dog):
	'''Draw dog platform'''
	mpl.rcParams['legend.fontsize'] = 10
	fix = dog.getFixPoints()
	knee = dog.getLegKnee()
	tcp = dog.getLegTcp()
	fig = plt.figure()
	ax = fig.gca(projection = '3d',xlim = [-300, 300], ylim = [-300,300], zlim = [-300, 300])
	for i in range(0,4):
		x = np.array([fix[i][0], knee[i][0], tcp[i][0]])
		y = np.array([fix[i][1], knee[i][1], tcp[i][1]])
		z = np.array([fix[i][2], knee[i][2], tcp[i][2]])
		ax.plot(x, y, z, c = 'r')
	ax.plot(fix[:, 0], fix[:, 1], fix[:, 2])
	ax.set_aspect('equal')
	plt.title('Four legs robot simulation')
	plt.show()

#example
#d = dogPlatform()
#tcp = np.array([[-150, 200, -130, 1],\
#		[250, 250, -130, 1],\
#		[-250, -250, -130, 1],\
#		[150, -200, -130, 1]])
#d.setLegTcp(tcp)
#print d.getLegPose()
#
#d.forwardKinematics()
#print d.getLegTcp()
#dogShow(d)
