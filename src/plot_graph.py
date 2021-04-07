import numpy as np
import matplotlib.pyplot as plt
from scipy import io

# pred_roll = np.genfromtxt("roll_predictions.txt",delimiter = ",")
# pred_pitch = -np.genfromtxt("yaw_predictions.txt",delimiter = ",")
# pred_yaw = np.genfromtxt("pitch_predictions.txt",delimiter = ",")

predictions = pred_roll = np.genfromtxt("predictions.txt",delimiter = ",")

vicon_data = io.loadmat('/home/ravi/Ravi_D/Acadamic/UPENN/Second_Sem/Learning_Robotics/HW/HW2/in_py/vicon/viconRot2.mat')
vicon_data_rots = vicon_data['rots']
vicon_roll = np.arctan2(vicon_data_rots[2, 1, :], vicon_data_rots[2, 2, :])
vicon_pitch = np.arctan2(-vicon_data_rots[2, 0,:],np.sqrt(vicon_data_rots[2, 1, :] ** 2 + vicon_data_rots[2, 2, :] ** 2))
vicon_yaw = np.arctan2(vicon_data_rots[1,0,:],vicon_data_rots[0,0,:])

# pred_roll[pred_roll >= np.pi] = -2*np.pi + pred_roll[pred_roll >= np.pi]
# pred_roll[pred_roll < -np.pi] = 2*np.pi + pred_roll[pred_roll < -np.pi]

# pred_pitch[pred_pitch > np.pi] = -2*np.pi + pred_pitch[pred_pitch > np.pi]
# pred_pitch[pred_pitch < -np.pi] = 2*np.pi + pred_pitch[pred_pitch < -np.pi]

# pred_yaw[pred_yaw > np.pi] = -2*np.pi + pred_yaw[pred_yaw > np.pi]
# pred_yaw[pred_yaw < -np.pi] = 2*np.pi + pred_yaw[pred_yaw < -np.pi]


# print(np.pi)
# print(np.max(pred_roll),np.min(pred_roll))
# print(np.max(pred_pitch),np.min(pred_pitch))
# print(np.max(pred_yaw),np.min(pred_yaw))
print(predictions[:,0].shape)
plt.figure(1)
plt.plot(vicon_roll,'k')
plt.plot(predictions[:,0])
plt.figure(2)
plt.plot(vicon_pitch,'k')
plt.plot(predictions[:,1])
plt.figure(3)
plt.plot(vicon_yaw, 'k')
plt.plot(predictions[:,2])
plt.show()