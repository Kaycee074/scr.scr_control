import libtmux

class Window():

	START_DIR = "~/catkin_ws"

	def __init__(self, name, command, start_dir = "~/catkin_ws"):
		self.name = name
		self.command = command
		self.start_dir = start_dir

	def create(self, session):
		window = session.new_window(attach = False,
									window_name = self.name,
									start_directory = Window.START_DIR)

		pane = window.split_window(attach = False)
		pane.send_keys(self.command)

'''
------------------------------------
'''

SESSION_NAME = "ROS"
TOF_DIR = "~/tof_control/SCR"

WINDOWS = [Window("roscore",     "roscore"),
	       Window("tof_c++",     "sudo tof", start_dir = TOF_DIR),
	       Window("octa_light",  "rosrun scr_control SCR_OctaLight_server.py"),
	       Window("blind",       "rosrun scr_control SCR_blind_server.py"),
	       Window("hvac",        "rosrun scr_control SCR_HVAC_server.py"),
	       Window("tof",         "rosrun scr_control SCR_TOF_server.py"),
	       Window("cos",         "rosrun scr_control SCR_COS_server.py")
		  ]
'''
------------------------------------
'''

if __name__ == "__main__":
	server = libtmux.Server()

	try:
		session = server.find_where({"session_name": SESSION_NAME})
		session.kill_session()
		print("Session '" + SESSION_NAME + "' already exists, deleting...")
	except:
		pass

	session = server.new_session(attach = False,
								 session_name = "ROS",
								)

	for w in WINDOWS:
		w.create(session)

	print("Services started on tmux session '" + SESSION_NAME + "'")