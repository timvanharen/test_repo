"""
32x24_to_matrix_v1.py

Author:  	Tim van Haren (timvanharen@live.com)
Date:		24 april, 2018
"""

import pygame
import serial
import time
import os

#constants
imageFeedWidth = 768 #1280
imageFeedHeight = 600 #768
xPixels = 32
yPixels = 24
pixelWidth 	=  imageFeedWidth // xPixels
pixelHeight = imageFeedHeight // yPixels

displayCoordinateX = 0
displayCoordinateY = 25
displayWidth = imageFeedWidth + 300 
displayHeight = imageFeedHeight 

# Set the coordinates at which the display window will be placed
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (displayCoordinateX, displayCoordinateY)

#pygame initialize for visualization
pygame.init()
screen = pygame.display.set_mode((displayWidth, displayHeight))

#User info text messages for on the screen
font = pygame.font.SysFont(None, 36)

#Serial communication initialize
ser = serial.Serial()
ser.port = 'COM3'
ser.baudrate = 115200

#variables and arrays defines
i				= 0
j				= 0
k				= 0
pixel			= 0
displayBlobs 	= 0
blobsFound 		= 0
done 			= False

inputBuffer 	= []
blobMinX 		= []
blobMinY 		= []
blobMaxX 		= []
blobMaxY 		= []

debugInfo		= ["blobCount : ", "minTemp : ", "maxTemp : ", "blobTemp "]
debugVal  		= []

#function to map temperature value of pixels to rgb values for temperature gradiënt
def convert_to_rgb(minimum, maximum, value):
    minimum, maximum = float(minimum), float(maximum)    
    halfmax = (minimum + maximum) / 2
    if minimum <= value <= halfmax:
        r = 0
        g = int( 255./(halfmax - minimum) * (value - minimum))
        b = int( 255. + -255./(halfmax - minimum)  * (value - minimum))
        return (r,g,b)    
    elif halfmax < value <= maximum:
        r = int( 255./(maximum - halfmax) * (value - halfmax))
        g = int( 255. + -255./(maximum - halfmax)  * (value - halfmax))
        b = 0
        return (r,g,b)

def display_welcome_message():
	# Welcome message secuence
	for i in range(0,128):
		msg1 = font.render("Infrared camera video processing, 32x24", True, (0, i, 0))
		msg2 = font.render("by Tim van Haren", True, (0, i, 0))
		screen.fill((0,0,0))
		screen.blit(msg1,(displayWidth//2 - msg1.get_width() // 2, displayHeight//2 - msg1.get_height() // 2 - 25))
		screen.blit(msg2,(displayWidth//2 - msg2.get_width() // 2, displayHeight//2 - msg2.get_height() // 2))
		pygame.display.flip()
		time.sleep(0.005)

	for i in range(0,128):
		msg1 = font.render("Infrared camera video processing, 32x24", True, (0, 128 - i, 0))
		msg2 = font.render("by Tim van Haren", True, (0, 128 - i, 0))
		screen.fill((0,0,0))
		screen.blit(msg1,(displayWidth//2 - msg1.get_width() // 2, displayHeight//2 - msg1.get_height() // 2 - 25))
		screen.blit(msg2,(displayWidth//2 - msg2.get_width() // 2, displayHeight//2 - msg2.get_height() // 2))
		pygame.display.flip()
		time.sleep(0.005)
		
def initialize_serial_comms():
	for i in range(0,128):
		msg = font.render("Synchronizing with MLX90640 sensor...", True, (0, i, 0))
		screen.fill((0,0,0))
		screen.blit(msg,(displayWidth//2 - msg.get_width() // 2, displayHeight//2 - msg.get_height() // 2))
		pygame.display.flip()
		time.sleep(0.005)
		
	# Initiate communications with the MLX90640
	ser.open()
	done = False
	while not done:
		while(ser.inWaiting() > 0):
			ser.read(1)
		print("cleared")
		if(ser.inWaiting() > 0):
			cmd = ser.read(1)
			if(cmd == b'R'):
				print("got R")
				ser.write(b'R\n')
				done = True
			elif(cmd == b'C'):
				print("got C")
				ser.write(b'C\n')
				while not done:
					if(ser.inWaiting() > 0):
						cmd = ser.read(1)
						if(cmd == b'R'):
							done = True
			else:
				while(ser.inWaiting() > 0):
					ser.read(1)
		else:
			time.sleep(0.01)
		
def check_key_pressed():
	# Check which key is pressed
	for event in pygame.event.get():
		# Quit command, Ctrl + c
		if event.type == pygame.QUIT:
			ser.close()
			return True
		
		# General user interface
		if event.type == pygame.KEYDOWN:
			# When x is pressed, quit.
			if event.key == pygame.K_x:
				ser.write(b'X\n')
				ser.close()
				screen.fill((0,0,0))
				msg = font.render("Closing application...", True, (0, 128, 0))
				screen.blit(msg,(displayWidth//2 - msg.get_width() // 2, displayHeight//2 - msg.get_height() // 2))
				pygame.display.flip()
				time.sleep(1)
				return True
				
			# When UP is pressed, display blobs.
			elif event.key == pygame.K_UP:
				displayBlobs = 1
				
			# When DOWN is pressed, hide blobs.
			elif event.key == pygame.K_DOWN:
				displayBlobs = 0
				
			# When p is pressed, pauze camera feed.
			elif event.key == pygame.K_p:
				# When pauze is enabled, MLX90640 can be programmed or read
				ser.close()
				pauze = 1
				msg = font.render(("Pauze"), True, (0, 128, 0))
				screen.blit(msg, (imageFeedWidth + 10, imageFeedHeight - 100))
				pygame.display.flip()
				
				while(pauze):
					for event in pygame.event.get():
						if event.type == pygame.KEYDOWN:
							# Resume the camera feed
							if event.key == pygame.K_p:
								pauze = 0
								screen.fill((0,0,0))
								ser.open()
								break
								
							# Quit
							elif event.key == pygame.K_x:
								pauze = 0
								ser.close()
								screen.fill((0,0,0))
								msg = font.render("Closing application...", True, (0, 128, 0))
								screen.blit(msg,(displayWidth//2 - msg.get_width() // 2, displayHeight//2 - msg.get_height() // 2))
								pygame.display.flip()
								time.sleep(1)
								return True

	return False

if __name__ == "__main__":
	
	#Start of program	
	display_welcome_message()

	initialize_serial_comms()
	done = False
	
	#Unending loop for processing incoming data arrays
	while not done:
		ser.write(b'R\n')
		#Wait for start message
		if(ser.inWaiting() > 0):
			cmd = ser.read(1)
			if(cmd == b'C'):
				initialize_serial_comms()
			if(cmd == b'R'):
				#write ack and ready message
				ser.write(b'R\n')
				waiting = 1
				while(waiting):
					val = ser.read(1)
					if(val == b'S'):
						waiting = 0

				if(ser.inWaiting() > 0 or val == b'S'):
					for i in range(0, 768):
						inputBuffer.append(int(float(ser.read(4))))
					
					cmd = ser.read(1)
					if(cmd == b'P'):
						#received the stop flag of the data string, only the IR data is displayed
						blobsFound = 0					
					elif(cmd ==	b'B'):
						debugVal.append(int(float(ser.read(2)))-10) # Amount of blobs found
						debugVal.append(100)#int(float(ser.read(4)))-1000) # lowest temperature measured
						debugVal.append(2500)#int(float(ser.read(4)))-1000) # highest temperature measured
						blobsFound = 1
						
						i = 0
						for i in range(0, debugVal[0]):
							blobMinX.append(int(float((ser.read(2))))-10)
							blobMaxX.append(int(float((ser.read(2))))-10)
							blobMinY.append(int(float((ser.read(2))))-10)
							blobMaxY.append(int(float((ser.read(2))))-10)
							cx = int((blobMaxX[i] - blobMinX[i]) / 2 + blobMinX[i])
							cy = int((blobMaxY[i] - blobMinY[i]) / 2 + blobMinY[i])
							
							debugVal.append(inputBuffer[cx * 32 + cy]) # blobTemp
							print("CX : CY : MinX : MaxX : MinY : MaxY : \r\n", cx, cy, blobMinX[i], blobMaxX[i], blobMinY[i], blobMaxY[i])
						
						cmd = ser.read(1)
						if(cmd == b'P'):
							pass #ok
						else:
							print("Oops?!?")
					else:
						print("stop command has not been received")
					
					screen.fill((0,0,0))
					
					#Drawing infrared pixels on the screen, the color corresponds from the measured temperature
					pixel = 0
					for i in range(0, yPixels):
						for j in range(0, xPixels):
							pygame.draw.rect(screen, convert_to_rgb(1000, 9999, inputBuffer[pixel]), pygame.Rect(j*pixelWidth, i*pixelHeight, pixelWidth, pixelHeight))
							pixel += 1
					
					#display dots in the center of pixels to indicate a detected hotspot/blob
					if blobsFound:
						#display associated data
						for i in range(0, 3):
							liveDebug = font.render((debugInfo[i] + str(debugVal[i])), True, (0, 128, 0))
							screen.blit(liveDebug, (imageFeedWidth + 10, 25 * (i) + 10))
						
						#display blob place and blob tempetature
						for i in range(0, debugVal[0]):
							pygame.draw.rect(screen, (255,255,255), pygame.Rect(
							blobMinY[i] * pixelWidth,  
							blobMinX[i] * pixelHeight, 
							(blobMaxY[i] - blobMinY[i]) * pixelWidth +  pixelWidth,
							(blobMaxX[i] - blobMinX[i]) * pixelHeight +  pixelHeight), 1)
							
							liveDebug = font.render((debugInfo[3] + str(i) + " : " + str(debugVal[i+3])), True, (0, 128, 0))
							screen.blit(liveDebug, (imageFeedWidth + 10, 25 * i + 100))
						
					del debugVal[:]
					del blobMinX[:]
					del blobMaxX[:]
					del blobMinY[:]
					del blobMaxY[:]
						
				else:
					time.sleep(0.1)
			else:
				print("something went wrong with serial communications, send restart command")
				time.sleep(1)
				Sync = 0
				timeout = 30
				while(Sync == 0 and timeout > 0):
					ser.write(b'F\n')
					while(ser.inWaiting() > 0):
						cmd = ser.read(1)
						print(cmd)
						if(cmd == b'A'):
							Sync = 1
					
					time.sleep(0.01)
					timeout -= 1
					
				time.sleep(0.01)
		pygame.display.flip()
		del inputBuffer[:]
		done = check_key_pressed()
		
	