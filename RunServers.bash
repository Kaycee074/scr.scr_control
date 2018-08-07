gnome-terminal -e roscore
gnome-terminal -e rosrun scr_control SCR_OctaLight_server.py
gnome-terminal -e rosrun scr_control SCR_HVAC_server.py
gnome-terminal -e rosrun scr_control SCR_COS_server.py
gnome-terminal -e rosrun scr_control SCR_TOF_server.py
gnome-terminal -e rosrun scr_control SCR_blind_server.py
gnome-terminal -e rosrun scr_control SCR_blind_server.py
gnome-terminal -e ./../../../tof_control/SCR/tof
