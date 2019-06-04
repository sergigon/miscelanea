#!/usr/bin/env python
# -*- coding: utf-8 -*-

def main():
	import rospy
	import requests
	import cv2
	'''
	# Descargar imagen
	url_imagen = "https://golang.org/doc/gopher/appenginegophercolor.jpg" # El link de la imagen
	nombre_local_imagen = "go.jpg" # El nombre con el que queremos guardarla
	imagen = requests.get(url_imagen).content
	with open(nombre_local_imagen, 'wb') as handler:
		handler.write(imagen)

	print('Downloaded image')
	'''

	filename = "go.jpg"
	# Final shape
	width = 480
	height = 360
	print('Image', 'height', 'width')

	rospy.sleep(2)
	# Read image
	oriimg = cv2.imread(filename,cv2.IMREAD_COLOR)
	# Get shape
	height_ori, width_ori, _ = oriimg.shape # Height x width
	print('Original', height_ori, width_ori)
	#cv2.imshow("oriimg",oriimg)

	# Apply Vertical crop (width mantained)
	print('Applying vertical crop')
	# Get vertical factor
	v_factor = float(width)/width_ori
	#print('Vertical factor', v_factor)
	# Get resized shape
	width_img = width
	height_img = int(v_factor*height_ori)

	if(height_img>=height): # Altura de imagen es mayor o igual que la final (SUCCESS)
		# Resize
		img_resized = cv2.resize(oriimg,(width_img,height_img))
		print('Image resized (with final width)', img_resized.shape)
		# Crop image
		img_cropped = img_resized[0:height, :] 
		print('Image cropped (with final width)', img_cropped.shape)

		cv2.imshow("Image resized (with final width)",img_resized)
		cv2.imshow("Image cropped (with final width)",img_cropped)
		k = cv2.waitKey()
		return
	else: # Altura de imagen es menor que la final (SUCCESS)
		print('Vertical crop fail')

	# Apply Horizontal crop (height mantained)
	print('Apllying horizontal crop')
	# Get horizontal factor
	h_factor = float(height)/height_ori
	#print('Horizontal factor', h_factor)
	# Get resized shape
	height_img = height
	width_img = int(h_factor*width_ori)
	# Resize
	img_resized = cv2.resize(oriimg,(width_img,height_img))
	print('Image resized (with final height)', img_resized.shape)
	# Crop image
	height_ori, width_ori, _ = oriimg.shape # Height x width
	img_cropped = img_resized[:, (width_img-width)/2:-(width_img-width)/2] 
	print('Image cropped (with final height)', img_cropped.shape)

	#cv2.imshow("Image resized (with final height)",img_resized)
	#cv2.imshow("Image cropped (with final height)",img_cropped)
	cv2.imwrite("thumbnail.png", img_cropped)
	print('finish')
	k = cv2.waitKey()
	return

	print('finish')
	k = cv2.waitKey()
	while(True):
		print('esperando...')
		rospy.sleep(2)

main()