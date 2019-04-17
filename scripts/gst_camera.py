import os, subprocess, StringIO
from enum import Enum



# Create an enum of different feed types
class FeedType (Enum):
	FT_Stereo	= 0
	FT_Arm		= 1
	FT_FoscamBlack	= 2
	FT_FoscamWhite	= 3




# DEFAULT VARIABLES ARE SET TO STEREOSCOPIC CAMS
# Image Format Variables
width = 720
height = 500
frame_rate = 30
img_format = "I420"

# Networking Variables
ip_end = 4
port = '5000'

# Camera Identification Variables
cur_feedType = FeedType.FT_Stereo
video_IDs = [1,2]
device_name = "Stereo Vision 2"
isUSB = True




# Returns the GST pipeline with updated variable values for Stereo Cam
def gst_pipeline_stereo():
	return "gst-launch-1.0 v4l2src device=/dev/video{} ! videoscale ! video/x-raw, width={}, height={}, framerate={}/1, format={} ! compositor name=comp sink_1::xpos={} ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.{} port={} v4l2src device=/dev/video{} ! videoscale ! video/x-raw, width={}, height={}, framerate={}/1, format={} ! comp.".format(video_IDs[0], width, height, frame_rate, img_format, width, ip_end, port, video_IDs[1], width, height, frame_rate, img_format)




# Returns the GST pipeline with updated variable values for Arm Cam
def gst_pipeline_arm():
	return "gst-launch-1.0 v4l2src device=/dev/video{} ! videoscale ! video/x-raw, width={}, height={}, framerate={}/1, format={} ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.{} port={}".format(video_IDs[0], width, height, frame_rate, img_format, ip_end, port)




# Returns the GST pipeline with updated variable values for Foscam (with IP)
def gst_pipeline_foscam(_foscamID):
	#return("gst-launch-1.0 rtspsrc location=rtsp://nova:rovanova@192.168.1.{}:88/videoMain ! autovideosink".format(_foscamID))
	return("gst-launch-1.0 rtspsrc location=rtsp://nova:rovanova@192.168.1.{}:88/videoMain ! decodebin ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.{} port={} sync=false".format(_foscamID, ip_end, port))




# Get port value, for having multiple streams
# For example, one stream of stereoscopic cams could be port 5000, and another stream could be 5001.
# Only the last digit will be changed, so use [0] for the first stream and [1] for the second.
#port_input = str(raw_input('Enter the final digit of the port: '))
#port = port[:3] + port_input[0]





# Get user input for camera feed type:
feed_input = str(raw_input(
	'Press (S) for Stereo Cam\n' +
	'Press (A) for Arm Cam\n' +
	'Press (B) for Black Foscam\n' +
	'Press (W) for Black Foscam\n' +
	'\t: ')).lower()

# Change the feed type depending on user input
if feed_input == 'a':
	# Change to Arm Camera feed
	cur_feedType = FeedType.FT_Arm
	device_name = 'HD USB Camera'
	width = 640
	height = 480
	port = '5001'

elif feed_input == 'b':
	# Change to Black Foscam feed
	cur_feedType = FeedType.FT_FoscamBlack
	device_name = 'Black Foscam'
	width = 640
	height = 480
	port = 5002
	isUSB = False

elif feed_input == 'w':
	# Change to White Foscam feed
	cur_feedType = FeedType.FT_FoscamWhite
	device_name = 'White Foscam'
	width = 640
	height = 480
	port = 5003
	isUSB = False





# Search /dev/ for video devices connected to rover
bash_cmd = "ls /dev/ | grep video"
output = subprocess.check_output(['bash','-c', bash_cmd])

# Create list of video device names
devs = []  
s = StringIO.StringIO(output)
for line in s:
	devs.append(line.strip("\n"))

 # Create list of video device ids
ids = []  
for i in range(len(devs)):  
	bash_cmd  = "cat /sys/class/video4linux/" + devs[i] + "/name "
	output = subprocess.check_output(['bash','-c', bash_cmd])
	ids.append(output.strip("\n"))



# Creates the list of desired camera devices from the dev list
devices = []
for idx, val in enumerate(ids):
	if val == device_name:
		devices.append(devs[idx].replace('video',''))
print('{} Device Index(es): {}\n'.format(device_name, devices))

# Update camera IDs
video_IDs = devices

print('Attempting to stream data from {} at IP = 192.168.1.{}:{}\n'.format(device_name, ip_end, port))




# Check for error in camera detection
if len(devices) > 0 or not isUSB:
	# Run the appropriate GST pipeline command
	if cur_feedType == FeedType.FT_Stereo:
		os.system(gst_pipeline_stereo())
	elif cur_feedType == FeedType.FT_Arm:
		os.system(gst_pipeline_arm())
	elif cur_feedType == FeedType.FT_FoscamBlack:
		os.system(gst_pipeline_foscam(53))
	elif cur_feedType == FeedType.FT_FoscamWhite:
		os.system(gst_pipeline_foscam(52))

# If camera not plugged in, output error message
else:
	print("Camera source '{}' not found. Please check to see if it is correctly plugged in.".format(device_name))
