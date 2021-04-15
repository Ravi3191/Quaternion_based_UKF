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

	#python error module indicates if it is unable to read from a file
	#load data
	predictions = np.genfromtxt(os.getcwd() +  "/data/predictions/predictions" + str(index) +".txt",delimiter = ",")
	vicon_data = np.genfromtxt(os.getcwd() + "/data/predictions/gt" + str(index) +".txt", delimiter = ',')
	vicon_time = np.genfromtxt(os.getcwd() + "/data/vicon/viconRaw" + str(index) + "_ts.txt", delimiter = ' ')
	predictions_time = np.genfromtxt(os.getcwd() + "/data/imu/imuRaw" + str(index) + "_ts.txt", delimiter = ' ')

	#plot the roll angles
	plt.figure(1)
	plt.plot(vicon_time,vicon_data[:,0],'k',label = 'vicon data')
	plt.plot(predictions_time,predictions[:,0],label = 'predictions')
	plt.title('roll pred vs ground truth')
	plt.legend()
	plt.xlabel('time')
	plt.ylabel('angle in rad')

	#plot the pitch angles
	plt.figure(2)
	plt.plot(vicon_time,vicon_data[:,1],'k',label = 'vicon data')
	plt.plot(predictions_time,predictions[:,1],label = 'predictions')
	plt.title('pitch pred vs ground truth')
	plt.xlabel('time')
	plt.ylabel('angle in rad')
	plt.legend()

	#plot the yaw angles
	plt.figure(3)
	plt.plot(vicon_time, vicon_data[:,2] ,'k',label = 'vicon data')
	plt.plot(predictions_time,predictions[:,2],label = 'predictions')
	plt.title('yaw pred vs ground truth')
	plt.legend()
	plt.xlabel('time')
	plt.ylabel('angle in rad')
	plt.show()

if __name__ == '__main__':
	main(sys.argv[1:])