import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import getopt

def main(argv):

	if(len(argv) == 0):
		print("Pass the file number as a command line argument");
		sys.exit(2)

	index = int(argv[0])

	if(index > 2 and index <= 0):
		print("The input argument shoule be either 1 or 2")
		sys.exit(2)

	#load data
	predictions = np.genfromtxt(os.getcwd() +  "/build/predictions" + str(index) +".txt",delimiter = ",")
	vicon_data = np.genfromtxt(os.getcwd() + '/data/vicon/viconRaw' + str(index) + '_rots.txt', delimiter = ' ')
	vicon_time = np.genfromtxt(os.getcwd() + '/data/vicon/viconRaw' + str(index) + '_ts.txt', delimiter = ' ')
	predictions_time = np.genfromtxt(os.getcwd() + '/data/imu/imuRaw' + str(index) + '_ts.txt', delimiter = ' ')

	#convert vicon data into euler angles for comparision
	vicon_data_rots = np.reshape(vicon_data,(3,3,vicon_data.shape[1]))
	vicon_roll = np.arctan2(vicon_data_rots[2, 1, :], vicon_data_rots[2, 2, :])
	vicon_pitch = np.arctan2(-vicon_data_rots[2, 0,:],np.sqrt(vicon_data_rots[2, 1, :] ** 2 + vicon_data_rots[2, 2, :] ** 2))
	vicon_yaw = np.arctan2(vicon_data_rots[1,0,:],vicon_data_rots[0,0,:])

	#plot the roll angles
	plt.figure(1)
	plt.plot(vicon_time,vicon_roll,'k',label = 'vicon data')
	plt.plot(predictions_time,predictions[:,0],label = 'predictions')
	plt.title('roll pred vs ground truth')
	plt.legend()
	plt.xlabel('time')
	plt.ylabel('angle in rad')

	#plot the pitch angles
	plt.figure(2)
	plt.plot(vicon_time,vicon_pitch,'k',label = 'vicon data')
	plt.plot(predictions_time,predictions[:,1],label = 'predictions')
	plt.title('pitch pred vs ground truth')
	plt.xlabel('time')
	plt.ylabel('angle in rad')
	plt.legend()

	#plot the yaw angles
	plt.figure(3)
	plt.plot(vicon_time, vicon_yaw ,'k',label = 'vicon data')
	plt.plot(predictions_time,predictions[:,2],label = 'predictions')
	plt.title('yaw pred vs ground truth')
	plt.legend()
	plt.xlabel('time')
	plt.ylabel('angle in rad')
	plt.show()

if __name__ == '__main__':
	main(sys.argv[1:])