import sys, time, os

sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "lights"))

import SCR_OctaLight_client as light_control


sum = 0 
iterations = 500

for i in range(iterations):
	start = time.time()
	light_control.sources_all(0, 0, 0, 0, 0, 0, 0, 0)
	duration = time.time() - start
	sum += duration

	start = time.time()
	light_control.sources_all(100, 0, 0, 0, 0, 0, 0, 0)
	duration = time.time() - start
	sum += duration

avg = sum/(iterations*2)
print(avg)