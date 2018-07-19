import sys, os, rospy, numpy, time
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "time_of_flight"))
import SCR_TOF_client as tof
import matplotlib.pyplot as plt

heatmap = numpy.asarray(tof.get_distances())
myPlot = plt.imshow(heatmap)

print("TEST")
while True:
	heatmap = numpy.asarray(tof.get_distances())
	myPlot.set_data(heatmap)
	plt.pause(.1)