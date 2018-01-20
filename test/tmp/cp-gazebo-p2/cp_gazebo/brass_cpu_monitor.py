#!/usr/bin/env python  

import os
import psutil
import rospy
from std_msgs.msg import Float64
import time


def find_gazebo():
	gazebo = None
	while gazebo is None:
		time.sleep(1)
		for proc in psutil.process_iter():
			try:
				if "gzserver" in proc.name:
					gazebo = proc
			except psutil.NoSuchProcess:
				pass
	return gazebo

def report_cpu_percentage():
	gazebo = find_gazebo()
	percent = psutil.cpu_percent(interval=None)
	gzP = gazebo.get_cpu_percent(interval=None)
	# For me, this returns 4
	CORES = len(psutil.cpu_percent(percpu=True))

	pub = rospy.Publisher("/energy_monitor/set_nuc_utilization", Float64, queue_size=10, latch=True)

	while not rospy.is_shutdown():
		time.sleep(2)
		# The overall percentage of CPU since the last time we called it
		percent = psutil.cpu_percent(interval=None)
		# The total cpu percentage of gz since last call, divide by number of cores
		gzP = gazebo.get_cpu_percent(interval=None) / CORES
		real_percent = percent - gzP
		rospy.loginfo('Percent = %s, GZ = %s, reporting = %s' %(percent, gzP,real_percent))
		msg = Float64()
		msg.data = real_percent
		pub.publish(msg)


if __name__ == "__main__":
	rospy.init_node('brass_cpu_monitor')
	report_cpu_percentage()