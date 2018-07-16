import sys, os, rospy, numpy, time
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "lights"))
import SCR_OctaLight_client as tof
import matplotlib.pyplot as plt

while True:
	state = tof.cct_all(1000, 1000)
	state = tof.cct_all(2000, 1000)
	'''
	heatmap = numpy.asarray(state)
	#time.sleep(.1)
	plt.imshow(heatmap)
	plt.pause(.001)
	'''
	print(state)
