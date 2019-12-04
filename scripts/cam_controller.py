###############################
######                   ######
###### Camera Controller ######
######                   ######
###############################

# This script connects to the GaCC script
#   and is able to control the cameras
#   via the command line.





###############################
###### IMPORTED PACKAGES ######
###############################

#!/usr/bin/env python

import gacc, os, sys, termios, tty, time


#######################
###### FUNCTIONS ######
#######################

# Prints a line across the terminal
def printLine (length = 50):
  print ('-' * length)

# Prints a formatted title on the terminal
def printTitle (title):
  printLine(14 + len(title))
  print('------ {} ------'.format(title.upper()))
  printLine(14 + len(title))


# Gets a single character input, instead of pressing eneter
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


# Asks for a string and validates for options
def askForString (prompt, multiChar, options = ['']):
  outputStr = None
  firstTime = True
  
  # Repeat until a valid string is reached
  while (outputStr == None):
    # Only print on the first time
    if firstTime:
      print '\n' + prompt + ' ',
      firstTime = False 
    
    # Use character detection or entire string
    if not multiChar:
      inputStr = getch()
    else:
      inputStr = raw_input()
    
    # Attempt to convert to string
    try:
      outputStr = str(inputStr)
      # Check if in valid options
      if not (outputStr.lower() in options or options == ['']):
        outputStr = None
    except:
      pass

  # Return the final string
  return outputStr.lower()




#######################
###### VARIABLES ######
#######################
isRunning = True # If the script is running
cur_menu = '0' # Current Menu 





##########################
###### MAIN PROGRAM ######
##########################

# Continue program until not running anymore
while (isRunning):

  # Clear window
  os.system('clear')

  # Main Menu
  if cur_menu == '0':
    printTitle('GStreamer and Camera Controller (GaCC)')
    print('\nPress (C) for Cameras')
    print('Press (N) for Networking')
    print('Press (H) for Help')
    print('Press (Q) to Quit\n')
    
    # Get the next command
    command = askForString('Please Enter a Command:', False, ['c','n','h','q','.'])
    
    # Change the menu based on the command
    if command == 'c':
      cur_menu = '1'
    elif command == 'n':
      cur_menu = '2'
    elif command == 'h':
      cur_menu = '3'
    elif command == 'q' or command == '.':
      isRunning = False
  
  
  # Cameras
  elif cur_menu[0] == '1':
    printTitle('Camera Controller')
    print('Current Quality Type: {}'.format(gacc.GetQualityType()))
    
    # Camera Menu
    if cur_menu == '1':
      print('\nPress (S) to Toggle a camera')
      print('Press (X) to Stop all cameras')
      print('Press (Q) to change Quality')
      print('Press (C) to List Connected cameras')
      print('Press (K) to List Known cameras')
      print('Press (.) to return Home\n')
      
      # Get the next command
      command = askForString('Please Enter a Command:', False, ['s','x','q','c','k','.'])
      
      # Change the menu based on the command
      if command == 's':
        cur_menu = '11'
      elif command == 'x':
        cur_menu = '12'
      elif command == 'q':
        cur_menu = '13'
      elif command == 'c':
        cur_menu = '14'
      elif command == 'k':
        cur_menu = '15'
      elif command == '.':
        cur_menu = '0'
    
    
    # Toggle Cameras
    elif cur_menu == '11':
    
      # Get the camera data
      camera_list = gacc.GetCameraData()
      
      print('\nTOGGLE A CAMERA STREAM:')
      
      # Create a button mapping dictionary
      buttonMap = {}
      
      # Print the list of camera names
      for camID in camera_list.keys():
        cam = camera_list[camID]
        
        # Check if camera is connected
        connected = gacc.IsCameraConnected(camID)
        if connected:
          connected = '*'
        else:
          connected = ' '
        
        # Check if camera is streaming already
        streaming = gacc.IsCameraStreaming(camID)
        if streaming:
          connected = '!'
        
        # Update the button mapper
        buttonMap[cam['KeyCode']] = camID
        
        # Output the camera option
        print('\t{}  Press ({}) for {}'.format(connected, cam['KeyCode'], camID))
        
      # Get List of camera keycodes
      keycodes = gacc.GetCameraData('KeyCode')
      keycodes.append('.')

      # Camera Selection
      cameraSelect = askForString('Please Select a Camera:', False, keycodes)
      
      # If a camera has been selected
      if cameraSelect != '.':
       gacc.StartStreaming(buttonMap[cameraSelect])
      # Otherwise return home
      else:
        cur_menu = '1'
    
    
    
    # Stop all camera streams
    elif cur_menu == '12':
      gacc.StopAllStreams()
      cur_menu = '1'
    
    
    
    
    # Quality Menu
    elif cur_menu == '13':
      # Get a New Quality
      print('\nQUALITY SETTINGS:')
      print('\nPress (H) to High quality')
      print('Press (M) to Medium quality')
      print('Press (L) to Low quality\n')
      print('Press (.) to go back\n')
      newQuality = askForString('Please Enter a new Quality Setting:', False, ['h','m','l','.'])
      if newQuality != '.':
        gacc.SetQualityType(newQuality)
      cur_menu = '1'
    
    
    # Connected Cameras Menu
    elif cur_menu == '14':
      # Get list of all connected cameras
      cams_connected = gacc.GetConnectedCameras()
      print("\nCONNECTED CAMERAS: {}\n".format(len(cams_connected)))

      # List out information for each camera
      # First, get the names of the cameras by the video* id
      print("Cameras by Name:")
      for cam in cams_connected:
        print("\tID: {}\tName: {}".format(cam[0], cam[1]))
      
      # Get the list of connected cameras by path
      pathCameras = gacc.GetCamerasByPath()
      print("\nCameras by Path on USB Hub:")
      for cam in pathCameras:
        print("\tPath: {}\tName: {}".format(cam[0], cam[1]))
      
      # Get a random command and change menu
      command = askForString('\nPress Anything to go back:', False)
      cur_menu = '1'
  
  
    # Known Cameras Menu
    elif cur_menu == '15':
      # Get the camera data
      camera_list = gacc.GetCameraData('ID')
      print('\nKNOWN CAMERAS: {}'.format(len(camera_list)))
      
      # Print the list of camera names
      for cam in camera_list:
        print('\t{}'.format(cam))
      
      # Print options
      print('\nPress (A) to Add a camera')
      print('Press (E) to Edit a camera')
      print('Press (D) to Delete a camera\n')
      print('Press (.) to go back\n')
      
      # Ask for next command
      command = askForString('Please Enter an Option:', False, ['a','e','d','.'])
      
      # Change the menu based on the command
      if command == 'a':
        cur_menu = '1'
      elif command == 'e':
        cur_menu = '1'
      elif command == 'd':
        cur_menu = '1'
      elif command == '.':
        cur_menu = '1'
  
  # Networking
  elif cur_menu[0] == '2':
    printTitle('Network Controller')
    print('Current IP Address: {}'.format(gacc.GetIPAddress()))
    print('Streaming Type: {}'.format(gacc.GetStreamingType()))
    
    # Networking Menu
    if cur_menu == '2':
      print('\nPress (I) to change the IP Address')
      print('Press (T) to change the Streaming Type')
      print('Press (.) to return Home\n')
      
      # Get the next command
      command = askForString('Please Enter a Command:', False, ['i','t','.'])
      if command == 'i':
        cur_menu = '21'
      if command == 't':
        gacc.ToggleStreamingType()
        cur_menu = '2'
      elif command == '.':
        cur_menu = '0'
    
    
    # Change IP Menu
    elif cur_menu == '21':
      # Get a New IP
      newIP = askForString('\nPlease Enter a new IP:', True)
      gacc.SetIPAddress(newIP)
      cur_menu = '2'
  
  
  # Help Menu
  elif cur_menu == '3':
    printTitle('Help Menu')
    print('\nPress (.) to return Home\n')
    
    # Get the next command
    command = askForString('Please Enter a Command:', False, ['.'])
    
    # Change the menu based on the command
    if command == '.':
      cur_menu = '0'
