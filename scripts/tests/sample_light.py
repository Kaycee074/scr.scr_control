import sys, time, os

sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "lights"))

import SCR_PentaLight_client as light_control

while True:
	light_control.ragbw(0, 0,     0, 0, 0, 0, 0)
	time.sleep(.1)
	light_control.ragbw(0, 0,     100, 0, 0, 0, 0)
	time.sleep(.1)
