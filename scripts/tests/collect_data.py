# Imports
import sys, time, os, datetime, threading
from pyowm import OWM
from pytz import timezone, utc

sys.path.append(os.path.join(os.path.dirname(__file__), "../lights"))
sys.path.append(os.path.join(os.path.dirname(__file__), "../blinds"))
sys.path.append(os.path.join(os.path.dirname(__file__), "../HVAC"))
sys.path.append(os.path.join(os.path.dirname(__file__), "../color_sensors"))
sys.path.append(os.path.join(os.path.dirname(__file__), "../time_of_flight"))

import SCR_OctaLight_client as light_control
import SCR_blind_client as blind_control
import SCR_HVAC_client as HVAC_control
import SCR_COS_client as COS_control
import SCR_TOF_client as TOF_control

# Format Timestamp as HH:MM:SS.MS
def format_time(sec):
	# ignore day/month/year
	sec = sec % (100 * 60 * 60)
	m, s = divmod(sec, 60)
	h, m = divmod(m, 60)
	d, h = divmod(h, 24)
	return "%02d:%02d:%02d.%02d" % (h, m, s, (sec*100)%100)

# Get HVAC Data as string
def HVAC_data():
	temp = HVAC_control.get_temp()
	rh = HVAC_control.get_rh()
	co2 = HVAC_control.get_co2()
	out = "temp: {t}\nrh: {r}\nco2: {c}\n".format(t=temp, r=rh, c=co2)
	return out

# Get Weather Data as string
def weather_data():
	owm = OWM('f38bad7ebd2079684d1bbe9fce79b11a')
	obs = owm.weather_at_coords(42.729173,-73.677731)
	w = obs.get_weather()
	temp = w.get_temperature('fahrenheit')['temp']
	pressure = w.get_pressure()['press']
	humidity = w.get_humidity()
	status = w.get_detailed_status()  
	return "temp: {t}\npressure: {p}\nhumidity: {h}\nstatus: {s}\n\n".format(t=temp, p=pressure, h=humidity, s=status)

# Collect Data for time
def collect_data(fun, args, delay, runtime, file, format):
	'''
	Collects data and writes to a file
	   Args: fun: function to call
	   		 args: arguments for fun
	   		 delay: time between data calls
	   		 runtime: total time to run data collection
	   		 file: name of file to write to
	   		 format: format of output
	   		 	<t> is replaced with timestamp
	   		 	<d> is replaced with data
	   		 	<dn> is replaced with data[n]
	'''


	# Initial Values
	out_file   = open(file, "w", 0)
	start_time = time.time()
	t = start_time

	# Until Data Collection is Complete
	while (time.time() - start_time < runtime):

		# Wait for delay
		while (time.time() - delay < t):
			pass

		# Get new data and timestamp
		t = time.time()
		timestamp = format_time(time.time())
		data = fun(*args)
		
		# Write data
		output = format.format(t=timestamp, d=data)
		out_file.write(output)

# Start collecting data in new thread
def collect_data_in_thread(fun, args, delay, runtime, file, format):
	t = threading.Thread(target=collect_data, args=(fun, args, delay, runtime, file, format))
	t.daemon = True
	t.start()
	return t


if __name__ == "__main__":

	#       (1 hour) * 24 hours
	period = 60 * 60 * 24


	#					   function                   args    delay  period  output              output format
	collect_data_in_thread(light_control.get_sources, [0, 0], 2,     period, "data/light.txt",   "{t}\n{d}\n")
	collect_data_in_thread(TOF_control.get_distances, [    ], 2,     period, "data/tof.txt",     "{t}\n{d}\n")
	collect_data_in_thread(COS_control.read_all,      [    ], 2,     period, "data/cos.txt",     "{t}\n{d}\n")
	collect_data_in_thread(HVAC_data,                 [    ], 60,    period, "data/hvac.txt",    "{t}\n{d}\n")
	collect_data_in_thread(weather_data,              [    ], 60,    period, "data/weather.txt", "{t}\n{d}\n")

	print("Data collection running for " + str(period//(60*60)) + " hours")
	print("Press Ctrl+C to cancel data collection")
	while True:
		pass