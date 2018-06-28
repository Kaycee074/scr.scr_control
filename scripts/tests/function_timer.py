import sys, time, os

sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "lights"))
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "blinds"))
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "HVAC"))
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "color_sensors"))
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), "time_of_flight"))

import SCR_PentaLight_client as light_control
import SCR_blind_client as blind_control
import SCR_HVAC_client as HVAC_control
import SCR_COS_client as COS_control
import SCR_TOF_client as TOF_control


functions_to_time = [
					#["ragbw        ",  light_control.ragbw,      [0, 0, 100, 0, 0, 0, 0]],
					#["ragbw_all    ",  light_control.ragbw_all,  [100, 0, 0, 0, 0]],
					#["cct          ",  light_control.cct,        [0, 0, 3000, 100]],
					#["cct_all      ",  light_control.cct_all,    [3000, 100]],
					#["get_lights   ",  light_control.get_lights, []],
					#["read         ",  COS_control.read,         [1]],
					#["read_all     ",  COS_control.read_all,     []],
					#["get_temp     ",  HVAC_control.get_temp,    []],
					["get_ep       ",  HVAC_control.get_ep,      []],
					["get_co2      ",  HVAC_control.get_co2,     []],
					["get_rh       ",  HVAC_control.get_rh,      []]
					]
iterations = 20

for f in functions_to_time:
	total = 0
	for i in range(iterations):
		start = time.time()
		f[1](*f[2])
		end = time.time()
		total += end - start
		time.sleep(.05)
	print(f[0] + ": " + str(total/iterations))