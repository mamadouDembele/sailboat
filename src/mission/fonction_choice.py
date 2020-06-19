import numpy as np
import math
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn import neighbors
import rospy
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

data = pd.read_csv('choice.csv')

xtrain=np.matrix(data['psi_wind']).T
ytrain=np.matrix(data['fonction']).T

knn = neighbors.KNeighborsClassifier(n_neighbors=3)
knn.fit(xtrain, ytrain)

def callback(data):
	data.

def main():
    rospy.init_node('node_choice_fonction')
    pub = rospy.Publisher('fonc_to_choose', String, queue_size=10)
    rospy.Subscriber("wind_angle", Quaternion, callback)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
    	msg = String()
    	msg.data=0
    	pub.publish()
    	rospy.spinOnce()
  		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

