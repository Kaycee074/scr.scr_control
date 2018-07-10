import sys, time, os
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "lights"))
import SCR_OctaLight_client as light_control

def changeLights(values):
	start = time.time()
	light_control.sources_all(*values)
	return time.time() - start

if __name__ == "__main__":
	s = 0
	iterations = 500
	for i in range(iterations):
		s += changeLights([100, 0, 0, 0, 0, 0, 0, 0])
		s += changeLights([0, 0, 0, 0, 0, 0, 0, 0])

	avg = s/(iterations*2)
	print("Average time to switch lights: " + str(avg))