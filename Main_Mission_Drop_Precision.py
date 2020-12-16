
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil 
import cv2
import numpy as np
import serial
import decimal
import string
import math

vehicle = connect('127.0.0.1:14551',wait_ready=True)
#arduino=serial.Serial('/dev/ttyACM0',9600, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)

img_width = 640
img_height = 480


camera_center = cv2.VideoCapture(0)
camera_center.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,img_width)
camera_center.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,img_height)

#======================== Parameter centering obj =================================

cx,cy,x,y,scx,scy=0,0,0,0,0,0
idx=0
idx2=0
idx3=0
idx4=0
idx5=0

range_center_x=15
range_center_y=15
center_x,center_y=0,0 
speed_centering=0.00075

#masukan jenis warna:4
drop_hl, drop_hh, drop_sl, drop_sh, drop_vl, drop_vh, = 146, 179, 0, 255, 0, 255 
#======================================= point coordinate ================================
lat_tolerant=0.0000100
lon_tolerant=0.0000100
alt_tolerant=0.2
alt_waypoint=0


#=========================================================================================

def str_to_int(s):
	try:
		return int(s)
	except ValueError:
		return s
		
def location (point,altt):
	point.alt=altt
	vehicle.simple_goto(point,0.5)
	while True:
		print ("GO TO NEXT POINT!")
		#print " GPS: %s" % vehicle.gps_0
		print " loc: %s" % vehicle.location.global_relative_frame
		'''
		obstacle=arduino.readline().strip('\n').strip('\r')
		if obstacle=='b':
			print ("OBSTACLE ADVOICE IS ON")
			vehicle.mode = VehicleMode("GUIDED")
			time.sleep(2)
			while(1):
				
		'''

		if  ((point.lat < (vehicle.location.global_relative_frame.lat+lat_tolerant)) and (point.lat > (vehicle.location.global_relative_frame.lat-lat_tolerant))) and ((point.lon < (vehicle.location.global_relative_frame.lon+lon_tolerant)) and (point.lon > (vehicle.location.global_relative_frame.lon-lon_tolerant))) and ((point.alt < (vehicle.location.global_relative_frame.alt+alt_tolerant)) and (point.alt > (vehicle.location.global_relative_frame.alt-alt_tolerant))):
			break
		if  vehicle.mode == 'LAND'or  vehicle.mode == 'RTL':
				#arduino.write('o')
				print ("FAIL SAVE ON")
				break
		if split==True:
			break

def rst_var():
	cx,cy,x,y=0,0,0,0
	idx=0
	idx2,idx3,idx4,idx5=0,0,0,0
	detect_cnt=0
	nothing=0
	detect=0
	center_x,center_y=0,0
	inversmode=False
	timemove=0
	
def arming_takeoff (aTargetAltitude):
   
    while not vehicle.is_armable:
        print("initialise...")
        time.sleep(1)

    print("Arming motors")
    
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    
    while not vehicle.armed:
		
		print("arming...")
		
		time.sleep(3)

    print("Bismillah Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  

    while True:
	
		print(" Ketinggian : ", vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.75:
			print("Selesai takeoff !")
			break
		
def mode_terbang (mod): #fungsi flighr mode

	if mod==1 :
		vehicle.mode = VehicleMode("GUIDED")
	elif mod==2:
		vehicle.mode = VehicleMode("LOITER")
	elif mod==3:
		vehicle.mode = VehicleMode("ALT_HOLD")
	elif mod==4:
		vehicle.mode = VehicleMode("STABILIZE")
	elif mod==5:
	    vehicle.mode = VehicleMode("LAND")
	elif mod==6:
		vehicle.mode = VehicleMode("RTL")
 
def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       
        0, 0,    
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        0b0000111111000111, 
        0, 0, 0, 
        velocity_x, velocity_y, velocity_z, 
        0, 0, 0, 
        0, 0)    
    # send command to vehicle on 1 Hz cycle
    
    vehicle.send_mavlink(msg)

def empty_callback(x):
    pass

def vision(angle):
	global cx,cy,x,y,new_x,new_y,c_x,c_y
	global idx, idx2,idx3,idx4,idx5
	global speed_centering
	global range_center_x, range_center_y
	#global P_x,P_y,I_x,I_y,D_x,D_y,Kp,Ki,error_x,error_y,max_error,last_error_x,last_error_y
	while (1):
	
		_, frame = camera_center.read()	
		hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		hul=cv2.getTrackbarPos('drop_hl', 'drop_windows')
		huh=cv2.getTrackbarPos('drop_hh', 'drop_windows')
		sal=cv2.getTrackbarPos('drop_sl', 'drop_windows')
		sah=cv2.getTrackbarPos('drop_sh', 'drop_windows')
		val=cv2.getTrackbarPos('drop_vl', 'drop_windows')
		vah=cv2.getTrackbarPos('drop_vh', 'drop_windows')
		cv2.putText(frame, "drop LAND MASK", (20,30),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0),2)
	
		lower = np.array([hul,sal,val])
		upper = np.array([huh,sah,vah])
		mask = cv2.inRange(hsv, lower, upper)
		#mask = cv2.medianBlur(mask,7)  #diblurkan
		erode_kernel = np.ones((3,3),np.uint8);
		eroded_img = cv2.erode(mask,erode_kernel,iterations = 1)
		dilate_kernel = np.ones((10,10),np.uint8);
		dilate_img = cv2.dilate(eroded_img,dilate_kernel,iterations = 1)
		res = cv2.bitwise_and(frame,frame, mask= dilate_img)
		contours,hierarchy = cv2.findContours(dilate_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	
		detect=len(contours)
		if detect>0:
			
			
			c= max(contours, key=cv2.contourArea)
			moment=cv2.moments(c)
			x= int (moment['m10']/moment['m00'])
			y= int (moment['m01']/moment['m00'])
			
			cx= int (x-320)*-1
			cy= int (y-240)*-1
			
			c_x=round (((cy)*speed_centering),3)
			c_y=round ((cx*speed_centering),3)
		
			new_x=round (((c_y*(math.sin(angle))) + (c_x*(math.cos(angle)))),1)        #inverse kninematics
			new_y=(round (((c_y*(math.cos(angle))) - (c_x*(math.sin(angle)))),1))*-1
			
			idx5=idx5+1
			if idx5>8:
				send_ned_velocity(new_x,new_y,0.1)
			'''
			if (cx<(cx+range_center_x) and cx>(cx-range_center_x)) and (cy<(cy+range_center_y) and cx>(cy-range_center_y)):
				idx=idx+1
					if idx>5:
						break
			'''
			for cnt in c:
				cv2.drawContours(frame,[cnt],-2,(200,0,0),10)
				cv2.circle(frame, (x,y),7,(0,255,0),-1)
		else :
		
				print ("not found")
				send_ned_velocity(0,0,0)
				idx5=0
				
			
		cv2.moveWindow('HASIL',400,550)
		cv2.moveWindow('FILTERING',10,550)
		cv2.imshow('HASIL',frame)
		cv2.imshow('FILTERING',res)
		k = cv2.waitKey(5) & 0xFF
		
		#------------------------------------------ to next steep
		altt_break=vehicle.rangefinder.distance
	   	if altt_break<1:
			break
		#if vehicle.location.global_relative_frame.alt <= 0.7 * 0.75:
		#	break
		#if k == 27:
		#	break
		
	cv2.destroyAllWindows()


def trackbars():
		cv2.namedWindow('drop_windows')
		cv2.createTrackbar('drop_hl', 'drop_windows',drop_hl,179,empty_callback)
		cv2.createTrackbar('drop_hh', 'drop_windows',drop_hh,179,empty_callback)
		cv2.createTrackbar('drop_sl', 'drop_windows',drop_sl,255,empty_callback)
		cv2.createTrackbar('drop_sh', 'drop_windows',drop_sh,255,empty_callback)
		cv2.createTrackbar('drop_vl', 'drop_windows',drop_vl,255,empty_callback)
		cv2.createTrackbar('drop_vh', 'drop_windows',drop_vh,255,empty_callback)



'''
==========================================================================================
								MAIN PROGRAM
==========================================================================================
'''

altitude= 3
home_point = LocationGlobalRelative(-7.7832048,110.2943042, 0)
drop_point = LocationGlobalRelative(-7.7832048,110.2943042, 0)
counter=0;
keyboard=raw_input ("Enter S for start mission ")
if keyboard in ('S','s') :
	arming_takeoff(altitude)
	trackbars()
	vision(vehicle.attitude.yaw)
vehicle.mode = VehicleMode("LAND")	
cv2.destroyAllWindows()
vehicle.close()	
print ("Mission Done!")






