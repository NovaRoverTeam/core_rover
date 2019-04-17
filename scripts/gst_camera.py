import os, subprocess, StringIO





# Image Format Variables
width = 720
height = 500
frame_rate = 30
img_format = "I420"

# Networking Variables
ip_end = 4
port = 5000

# Camera Identification Variables
video_IDs = [1,2]
device_name = "Stereo Vision 2"




# Returns the GST pipeline with updated variable values
def gst_pipeline():
	return "gst-launch-1.0 v4l2src device=/dev/video{} ! videoscale ! video/x-raw, width={}, height={}, framerate={}/1, format={} ! compositor name=comp sink_1::xpos={} ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.{} port={} v4l2src device=/dev/video{} ! videoscale ! video/x-raw, width={}, height={}, framerate={}/1, format={} ! comp.".format(video_IDs[0], width, height, frame_rate, img_format, width, ip_end, port, video_IDs[1], width, height, frame_rate, img_format)



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


# Creates the list of camera devices from the dev list
devices = []
for idx, val in enumerate(ids):
	if val == device_name:
		devices.append(devs[idx].replace('video',''))
print('{} Device Index(es): {}\n'.format(device_name, devices))

# Update camera IDs
video_IDs = devices


# Run the GST pipeline command
os.system(gst_pipeline())
