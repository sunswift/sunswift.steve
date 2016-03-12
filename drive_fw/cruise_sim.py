import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import numpy as np
import math as math

#Regen
REGEN_DISABLED = True

#DEM GAINZ
PROPORTIONAL_GAIN = 0.01
INTEGRAL_GAIN = 0.0001
DERIVATIVE_GAIN = 0

#WSC GAINS
#P = 0.01
#I = 0.0001
#D = 0

#Mass of Car
MASS = 400

#Phase limit set on wavesculptors. 3 Phases each motor. Double if using dual motors
#Thus, 60A phase limit set on Wavesculptors = 360A
PHASE_LIMIT = 360

#Nm per unit phase current
TQ_CONSTANT = 0.44

#Diameter of wheelmotors - check with mech
WHEEL_DIAMETER = 0.558

#CdA
#Drag Equation - Fd = (1/2)*p*v^2*CdA
#p is the density of 
CDA = 0.17

#Density of air at 15 degrees.
DENSITY_OF_AIR = 1.225

#Rolling resistance constant. 
CRR = 0.010

#Interval between samples - 10 Hz
INTERVAL = 0.1

#CAR MODEL GLOBALS
old_velocity = 0.0
current_velocity = 0.0
iteration = 0
cruise_integral = 0.0
cruise_proportional = 0.0
cruise_derivative = 0.0
error = 0.0
incline = 0.0

#Seconds to run sim
DURATION = 400
NUM_ITERATIONS = (int)(DURATION/INTERVAL)

PI = 3.1416

def main():

	global current_velocity
	global old_velocity
	global cruise_integral

	#25 m/s is 90km/h
	#velocity = kph2mps(100)
	#drag = (1.0/2)*DENSITY_OF_AIR*(velocity**2)*CDA
	#print drag_force(30)
	
	car_vel = [0]*NUM_ITERATIONS
	car_vel_kph = [0]*NUM_ITERATIONS
	cruise_integral_tq = [0]*NUM_ITERATIONS
	cruise_proportional_tq = [0]*NUM_ITERATIONS
	cruise_derivative_tq = [0]*NUM_ITERATIONS
	cruise_torque = [0]*NUM_ITERATIONS
		
	
	incline = 0.0
	#itnum = 0

	#Set Starting Velocity - set at 100km/hr
	current_velocity = kph2mps(100)
	old_velocity = kph2mps(100)
	
	#Set Starting Cruise Set Speed RPM
	cruise_rpm = mps2rpm(kph2mps(100))
	
	#Initialise Cruise Integral - may choose to have model info here
	cruise_integral = 0
	
	for i in range(0,NUM_ITERATIONS):
	
		#For the derivative term
		current_rpm = mps2rpm(current_velocity)
		old_rpm = mps2rpm(old_velocity)

		cruise_torque[i] = calculate_torque(cruise_rpm, current_rpm, old_rpm)
		
		cruise_integral_tq[i] = cruise_integral
		cruise_proportional_tq[i] = cruise_proportional
		cruise_derivative_tq[i] = cruise_derivative

		old_velocity = current_velocity
		car_vel[i] = current_velocity
		car_vel_kph[i] = mps2kph(current_velocity)
		
		car_model(cruise_torque[i]*PHASE_LIMIT, incline)
		
		#Incline change at 300 seconds
		if i == 50/INTERVAL:
			incline = 2.0
			#cruise_rpm = mps2rpm(kph2mps(92))
			
		if i == 100/INTERVAL:
			incline = -2.0
			
		if i == 150/INTERVAL:
			incline = 0.0
			
		if i == 200/INTERVAL:
			cruise_rpm = mps2rpm(kph2mps(101))
			
		
			
		#if i == 7000:
		#	incline = 1.0
		
	fig = plt.figure(0, figsize=(6,6))
	fig.patch.set_facecolor('white')
	ind = np.arange(len(car_vel_kph))
	ind = ind*INTERVAL
	plt.plot(ind, car_vel_kph, 'blue', linewidth=1.5)
	plt.ylabel('velocity [kph]', fontsize=16)
	plt.xlabel('seconds', fontsize=16)
	plt.grid(True)
	plt.draw()
	

	fig = plt.figure(1, figsize=(6,6))
	fig.patch.set_facecolor('white')
	ind = np.arange(len(car_vel_kph))
	ind = ind*INTERVAL
	plt.plot(ind, cruise_integral_tq, 'blue', linewidth=1.5)
	plt.plot(ind, cruise_proportional_tq, 'red', linewidth=1.5)
	plt.plot(ind, cruise_derivative_tq, 'black', linewidth=1.5)
	plt.plot(ind, cruise_torque, 'green', linewidth=1.5)
	plt.ylabel('cruise states', fontsize=16)
	plt.xlabel('seconds', fontsize=16)
	plt.grid(True)
	plt.draw()
	
	
	plt.show()
	
	print kph2mps(1)
	print mps2rpm(kph2mps(140))
	
def calculate_torque(cruise_rpm, current_rpm, old_rpm):
	global error
	global cruise_integral
	global cruise_proportional
	global cruise_derivative
	global current_velocity
	global old_velocity
	
	current_rpm = mps2rpm(current_velocity)
	
	error = cruise_rpm - current_rpm
	print "ERROR: " + str(error) + "\n\r"
	cruise_proportional = error * PROPORTIONAL_GAIN
	cruise_integral = cruise_integral + error * INTEGRAL_GAIN
	cruise_derivative = (current_rpm - old_rpm) * DERIVATIVE_GAIN
	
	#cruise_proportional = absLimit(cruise_proportional, 1.0)
	cruise_integral = absLimit(cruise_integral, 1.0)
	
	#Limit Integral to above 0 (no regen)
	if REGEN_DISABLED == True and cruise_integral <= 0.0:
		cruise_integral = 0.0
	
	torque = cruise_proportional + cruise_integral + cruise_derivative
	
	#Limit torque to above 0 (no regen)
	if REGEN_DISABLED == True and torque <= 0.0:
		torque = 0.0
	
	torque = absLimit(torque, 1.0)
	
	return torque
	
def absLimit(limitee, limit_value):
	if limitee >= limit_value:
		limitee = limit_value
	if limitee <= -limit_value:
		limitee = -limit_value
	
	return limitee

	
def drag_force(velocity):
	drag = (1.0/2)*DENSITY_OF_AIR*(velocity**2)*CDA
	return drag
	
def rolling_force():
	rolling = MASS*9.8*CRR
	return rolling
	
def power(force, velocity):
	power = force * velocity;
	return power

def kph2mps(kph):
	mps = (kph*1000.0)/(60*60)
	return mps
	
def mps2kph(mps):
	kph = (mps*60*60)/1000.0
	return kph
	
def mps2rpm(mps):
	rpm = 60.0 * (mps / (PI * WHEEL_DIAMETER))
	return rpm

def rpm2mps(rpm):
	mps = (rpm/60.0) * (PI * WHEEL_DIAMETER)
	return mps

def torque2force(torque):
	force = torque/(WHEEL_DIAMETER/2.0)
	return force

def current2torque(current):
	torque = current*TQ_CONSTANT
	return torque
	
def currentlimit(current):
	if current >= PHASE_LIMIT:
		current = PHASE_LIMIT
	if current <= -PHASE_LIMIT:
		current = -PHASE_LIMIT

	return current
	
def car_model(current, incline):

	global current_velocity
	
	#no regen
	#if torque2force(current2torque(current)) >= 0.0:
	drive_force = torque2force(current2torque(current))
	#else :
	#	drive_force = 0.0
	print "Phase Current: " + str(current/6)
	print "Drive Force: " + str(drive_force)
	
	#Add in rolling resistance.
	resist_force = drag_force(current_velocity) + rolling_force()
	print "Resist Force: " + str(resist_force)
	
	#Add in inclines
	resist_force += MASS*9.8*math.sin(math.radians(incline))
	
	net_force = drive_force - resist_force
	print "Net Force: " + str(net_force)
	
	
	print "Old Current Velocity: " + str(mps2kph(current_velocity))
	current_velocity = current_velocity + (net_force*INTERVAL/MASS)
	print "New Current Velocity: " + str(mps2kph(current_velocity))
	
if __name__ == '__main__':
	main()