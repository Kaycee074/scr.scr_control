import sys, os, rospy, numpy, time
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "time_of_flight"))
import SCR_TOF_client as tof
import matplotlib.pyplot as plt

while True:
	state = tof.getHeatmap_client()
	heatmap = numpy.asarray(state.distances)
	heatmap = numpy.reshape(heatmap, (len(state.distances)/state.room_length, state.room_length))
	plt.imshow(heatmap)
	plt.pause(0.1)
