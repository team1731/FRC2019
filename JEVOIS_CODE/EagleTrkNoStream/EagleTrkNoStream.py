import libjevois as jevois
import cv2
import numpy as np
import json
import math
import time

#Try termite instead of PuTTY
#Holders for target data
pixels = [(0,0), (0,0), (0,0), (0,0)]
allTargets = {} #Dictionaries https://stackoverflow.com/questions/10487278/how-to-declare-and-add-items-to-an-array-in-python
target = {}


# Values to use in target distance calculation
Ta = 12 #Actual target width in inches. This variable is never used. Why is this here?
FOV = 50  #Camera View in degrees. JeVois = 65.0, Lifecam HD-300 = 68.5, Cinema = 73.5

##Threshold values for Trackbars, These are pulled from the CalFile
'''
rawCalFile = open ('Calibration')
readCalFile = rawCalFile.read()
CalFile = readCalFile.split(",")
uh = int(CalFile[0])
lh = int(CalFile[1])
us = int(CalFile[2])
ls = int(CalFile[3])
uv = int(CalFile[4])
lv = int(CalFile[5])
er = int(CalFile[6])
dl = int(CalFile[7])
ap = int(CalFile[8])
ar = int(CalFile[9])
sl = float(CalFile[10])

rawCalFile.close() #Close calibration file
'''
#CalFile keeps getting reset... just set these
uh = 90
lh = 70
us = 255
ls = 150
uv = 255
lv = 200
er = 0
dl = 2
ap = 5
ar = 100
sl = 1.0

class ImageObject:
	def __init__(self, _xPos, _yPos, _area, _angle, _side):
		self.xPos = _xPos
		self.yPos = _yPos
		self.area = _area
		self.angle = _angle
		self.side = _side


class EagleTrkNoStream:
	# ###################################################################################################
	## Constructor
	def __init__(self):
		# Instantiate a JeVois Timer to measure our processing framerate:
		self.timer = jevois.Timer("Catbox", 100, jevois.LOG_INFO)
		
	# ###################################################################################################
	## Process function without USB output
	def processNoUSB(self, inframe):
		# Get the next camera image (may block until it is captured) and here convert it to OpenCV BGR by default. If
		# you need a grayscale image instead, just use getCvGRAY() instead of getCvBGR(). Also supported are getCvRGB()
		# and getCvRGBA():
		
		inimg = inframe.getCvBGR()
		
		normalizedY = 0
		normalizedZ = 0
		
		# Start measuring image processing time (NOTE: does not account for input conversion time):
		#self.timer.start()
		myTimer = time.time()
		#Convert the image from BGR(RGB) to HSV
		hsvImage = cv2.cvtColor( inimg, cv2.COLOR_BGR2HSV)
		
		## Threshold HSV Image to find specific color
		binImage = cv2.inRange(hsvImage, (lh, ls, lv), (uh, us, uv))
		
		# Erode image to remove noise if necessary.
		binImage = cv2.erode(binImage, None, iterations = er)
		#Dilate image to fill in gaps
		binImage = cv2.dilate(binImage, None, iterations = dl)
		
		##Finds contours (like finding edges/sides), 'contours' is what we are after
		#im2 is old, don't use
		
		contours, hierarchy = cv2.findContours(binImage, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_KCOS)
		
		##arrays to will hold the good/bad polygons
		# ^^^ Huh?
		
		images = []
		#images2 = []
		squares = []
		badPolys = []
		pairs = []

		index = 0
		## Parse through contours to find allTargets
		for c in contours:
			if (contours != None) and (len(contours) > 0):
				cnt_area = cv2.contourArea(c)
				hull = cv2.convexHull(c , 1)
				hull_area = cv2.contourArea(hull)  #Used in Solidity calculation
				p = cv2.approxPolyDP(hull, ap, 1)
				
				if (cv2.isContourConvex(p) != False) and (len(p) == 4) and (cv2.contourArea(p) >= ar):
					filled = cnt_area/hull_area
					if filled <= sl:
						
						index = index+1
						br = cv2.boundingRect(p)
						x = br[0] + (br[2]/2)
						y = br[1] + (br[3]/2)
						
						rect = cv2.minAreaRect(p)
						box = cv2.boxPoints(rect)
						box = np.int0(box)
						
						color = (0,0,255)
						
						#if rect[1][0] > rect[1][1]:
							#color = (255,0,0)
						
						#cv2.drawContours(inimg,[box],0,color,1)
						#cv2.putText(inimg, str(index), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255),1, cv2.LINE_AA)                                
						images.append(ImageObject(x, y, cv2.contourArea(c), rect[2], 0))
						squares.append(p)
						#images2.append(ImageObject(x, y, cv2.contourArea(c), rect[2], 0))
						
				else:
					badPolys.append(p)
		
		##BoundingRectangles are just CvRectangles, so they store data as (x, y, width, height)
		##Calculate and draw the center of the target based on the BoundingRect

		#If the width is bigger (minAreaRect[1][0]), then it's a left
		#If the height is bigger (minAreaRect[1][1]), then it's a right
		
		processedSquares = []
		squares2 = squares
		for squar in squares:
			rightMostSquare = None
			for s in squares2:
				if rightMostSquare == None:
					rightMostSquare = s
				else:
					rect = cv2.minAreaRect(rightMostSquare)
					thisRect = cv2.minAreaRect(s)
					if thisRect[0][0] > rect[0][0]:
						rightMostSquare = s
			processedSquares.append(rightMostSquare)
			newSquares2 = []
			for s in squares2:
				if not cv2.minAreaRect(s)[0][0] == cv2.minAreaRect(rightMostSquare)[0][0]:
					newSquares2.append(s)
			squares2 = newSquares2
		
		processedIndex = 0
		for ps in processedSquares:
			x = cv2.minAreaRect(ps)[0][0]
			y = cv2.minAreaRect(ps)[0][1]
			#cv2.putText(inimg, str(processedIndex), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255),1, cv2.LINE_AA) 
			processedIndex = processedIndex+1
		
		squares = processedSquares
		
		if len(squares) > 0:
			firstSquare = cv2.minAreaRect(squares[0])
		
			if firstSquare[1][0] > firstSquare[1][1]:
				del squares[0]
		
		rightSquare = None
		
		for s in squares:
			rect = cv2.minAreaRect(s)
			
			if rightSquare == None:
				if rect[1][0] < rect[1][1]:
					rightSquare = s
			elif rect[1][0] > rect[1][1]:
				pairs.append((s, rightSquare))
				rightSquare = None
			else:
				rightSquare = None
				
		middleMostPair = None
		middleMostCentroid = (0, 0)
		
		for pair in pairs:
			#currentPairCentroid = compute centroid of pair (pair0.x + pair1.x)/2
			#if middleMostPair is None:
			
			#if abs(currentPairCentroid - 160) <
			
			if middleMostPair == None:
				thisPair0X = cv2.minAreaRect(pair[0])[0][0]
				thisPair1X = cv2.minAreaRect(pair[1])[0][0]
				thisPairXCentroid = (thisPair0X+thisPair1X)/2
				thisPair0Y = cv2.minAreaRect(pair[0])[0][1]
				thisPair1Y = cv2.minAreaRect(pair[1])[0][1]
				thisPairYCentroid = (thisPair0Y+thisPair1Y)/2
				
				middleMostPair = pair
				middleMostCentroid = (thisPairXCentroid, thisPairYCentroid)
			else:
				#160
				thisPair0X = cv2.minAreaRect(pair[0])[0][0]
				thisPair1X = cv2.minAreaRect(pair[1])[0][0]
				thisPairXCentroid = (thisPair0X+thisPair1X)/2
				thisPair0Y = cv2.minAreaRect(pair[0])[0][1]
				thisPair1Y = cv2.minAreaRect(pair[1])[0][1]
				thisPairYCentroid = (thisPair0Y+thisPair1Y)/2
			
				pair0X = cv2.minAreaRect(middleMostPair[0])[0][0]
				pair1X = cv2.minAreaRect(middleMostPair[1])[0][0]
				pairXCentroid = (pair0X+pair1X)/2
				
				if abs(thisPairXCentroid - 160) < abs(pairXCentroid - 160):
					middleMostPair = pair
					middleMostCentroid = (thisPairXCentroid, thisPairYCentroid)
		
		
		#cv2.circle(inimg, (int(middleMostCentroid[0]), int(middleMostCentroid[1])), 10, (255,0,0), 1)
		
		normalizedY = 0
		normalizedZ = 0
		
		if not middleMostCentroid == (0, 0):
			focalLength = 320/(2 * math.tan(math.radians(FOV)/2)) #320 is image width
			normalizedY = -(middleMostCentroid[0] - (320/2 - 0.5)) / focalLength
			normalizedZ = (middleMostCentroid[1] - (240/2 - 0.5)) / focalLength #240 is image height
		'''
		leftImage = 0
		rightImage = 0
		i = 0
		for image in images2:
			
			if leftImage != 0 and rightImage != 0:
				# Store this as a target
				theX = (leftImage.xPos+rightImage.xPos)/2
				theY = (leftImage.yPos+rightImage.yPos)/2
				cv2.circle(inimg, (int(theX), int(theY)), 5, (255,0,0), 2)
				leftImage = 0
				rightImage = 0
				
			if leftImage == 0 and image.angle < -60:
				# Leftside image
				if rightImage != 0:
					if rightImage.xPos < image.xPos:
						# Discard this, it's on the wrong side.
					else:
						leftImage = ImageObject(image.xPos, image.yPos, image.area, image.angle, image.side)
				else:
					leftImage = ImageObject(image.xPos, image.yPos, image.area, image.angle, image.side)
			else:
				if leftImage != 0:
					if leftImage.xPos > image.xPos:
						# Discard this, it's on the wrong side.
					else:
						rightImage = ImageObject(image.xPos, image.yPos, image.area, image.angle, image.side)
				else:
					rightImage = ImageObject(image.xPos, image.yPos, image.area, image.angle, image.side)
				
			cv2.putText(inimg, str(i), (image.xPos, image.yPos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1, cv2.LINE_AA)
			i = i+1
			# Check angles and make centers for each found target
			
		biggestImages = []
		
		r = 3
		if len(images) < r:
			r = len(images)
			
		for i in range(r):
			bigImage = None
			for image in images:
				if bigImage == None:
					bigImage = image
				elif image.area > bigImage.area:
					bigImage = image
		
			biggestImages.append(bigImage)
			images.remove(bigImage)
		target = {}
		allTargets = {}
		biggestImagesY = []
		
		leftSides = 0
		rightSides = 0
		
		for image in biggestImages:
			
			# Check angles and distance and remove the image that is not like the others
			# cv2.circle(inimg, (int(image.xPos), int(image.yPos)), 5, (255,0,0), 2)
			biggestImagesY.append(image.yPos)
			if(image.angle < -60):
				leftSides += 1
				image.side = 1
			else:
				rightSides += 1
				image.side = 2
			
		biggestImagesY.sort()
		
		sideToKill = 0
		
		if(leftSides >= 2):
			sideToKill = 1
		else:
			sideToKill = 2
		
		if len(biggestImages) > 2:
			if biggestImages[0].xPos+biggestImages[1].xPos > biggestImages[1].xPos+biggestImages[2].xPos:
				if biggestImages[0].side == sideToKill:
					del biggestImages[0]
				else:
					del biggestImages[1]
			else:
				if biggestImages[1].side == sideToKill:
					del biggestImages[1]
				else:
					del biggestImages[2]
				
		if len(biggestImages) > 2:
			i = 0
			for image in biggestImages:
				if image.yPos == biggestImagesY[0]:
					del biggestImages[i]
					break
				i = i+1
			
		
		#for image in biggestImages:
			#cv2.circle(inimg, (int(image.xPos), int(image.yPos)), 15, (0,255,255), 2)
		
		if len(biggestImages) == 2:
			XCenter = (biggestImages[0].xPos+biggestImages[1].xPos)/2
			YCenter = (biggestImages[0].yPos+biggestImages[1].yPos)/2
			
			focalLength = 320/(2 * math.tan(math.radians(FOV)/2)) #320 is image width
			oldnormalizedY = (XCenter - 320/2 - 0.5) / focalLength
			oldnormalizedZ = -(YCenter - 240/2 - 0.5) / focalLength #240 is image height
			
			
			outputPixels = str(normalizedY)+","+str(normalizedZ)
			#cv2.circle(inimg, (int(XCenter), int(YCenter)), 5, (0,255,0), 1)

		for s in squares:
			
			br = cv2.boundingRect(s)
			rect = cv2.minAreaRect(s)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			
			cv2.drawContours(inimg,[box],0,(255,0,0),1)
			cv2.putText(inimg, "Angle: "+str(round(rect[2])), (br[0], br[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255,255,255),1, cv2.LINE_AA)
			
			#Target "x" and "y" center 
			x = br[0] + (br[2]/2)
			y = br[1] + (br[3]/2)
			
			allTargets.update({"x"+str(i) : x})
			allTargets.update({"y"+str(i) : y})
			allTargets.update({"a"+str(i) : rect[2]})
			
			cv2.rectangle(inimg, (br[0],br[1]),((br[0]+br[2]),(br[1]+br[3])),(0,255,255), 1,cv2.LINE_AA)
			
			# i += 1
			
			# Oh, I need to set an expression if I want my if statement to work
			
		
		cv2.putText(inimg, "Tracking "+str(len(biggestImages))+" suspected targets", (3, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1, cv2.LINE_AA)
		
		for s in squares:
			if len(squares) > 0:
				#Build "pixels" array to contain info desired to be sent to RoboRio
				pixels = {"Trk" : 1, "XCntr" : x, "YCntr" : y}
				json_pixels = json.dumps(pixels)
				
				
				cv2.putText(inimg, "Tracking", (3, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1, cv2.LINE_AA)
				cv2.putText(inimg, str(x), (3, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1, cv2.LINE_AA)
				cv2.putText(inimg, str(y), (3, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1, cv2.LINE_AA)
				cv2.rectangle(inimg, (br[0],br[1]),((br[0]+br[2]),(br[1]+br[3])),(0,255,255), 2,cv2.LINE_AA)
				
		
		
		
		if not squares:
			allTargets = {}
			target = {}
			# pixels = {"Trk" : 0, "XCntr" : 0, "YCntr" : 0}
			# json_pixels = json.dumps(pixels)
			# cv2.putText(inimg, "Not Tracking", (3, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1, cv2.LINE_AA)
			
		'''
		outimg = inimg
		
		# Write frames/s info from our timer into the edge map (NOTE: does not account for output conversion time):
		#fps = self.timer.stop()
		#height, width, channels = outimg.shape # if outimg is grayscale, change to: height, width = outimg.shape
		pixels = {"DeltaTime" : str((time.time()-myTimer)), "Y" : str(normalizedY), "Z" : str(normalizedZ)}
		jevois.sendSerial(json.dumps(pixels))
		#cv2.putText(outimg, fps, (3, height - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
		
		
		# Convert our BGR output image to video output format and send to host over USB. If your output image is not
		# BGR, you can use sendCvGRAY(), sendCvRGB(), or sendCvRGBA() as appropriate:
		# json_output = json.dumps(allTargets)
		
		#jevois.sendSerial(json_output)
		#outframe.sendCvBGR(outimg,50)
		
		
