import numpy as np 
import cv2
import matplotlib.pyplot as plt

def get_image():
    webcam = cv2.VideoCapture('http://192.168.100.16:8080/video')
    check, frame = webcam.read()
    cv2.imwrite(filename='saved_img.jpg', img=frame)
    webcam.release()
    return frame

# def filter_image():
    
def get_obstacle_vertices():
    # Reading image 
    img2 = cv2.imread('saved_img.jpg', cv2.IMREAD_COLOR) 
    #img2 = dilation  

    # Reading same image in another  
    # variable and converting to gray scale. 
    img = cv2.imread('saved_img.jpg', cv2.IMREAD_GRAYSCALE) 
    #img = dilation   

    # Converting image to a binary image 
    # ( black and white only image). 
    _, threshold = cv2.threshold(img, 110, 255, cv2.THRESH_BINARY) 
  
    # Detecting contours in image. 
    contours, _= cv2.findContours(threshold, cv2.RETR_TREE, 
                                  cv2.CHAIN_APPROX_SIMPLE) 

    vertices_array = [[0]]
    # Going through every contours found in the image. 
    k = 0
    for cnt in contours : 

        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True) 
  
        # draws boundary of contours. 
        cv2.drawContours(img2, [approx], 0, (0, 0, 255), 5)  
  
        # Used to flatted the array containing 
        # the co-ordinates of the vertices. 
        n = approx.ravel()    
        i = 0
        polygon = [[0,0]]
        for j in n : 
            if(i % 2 == 0): 
                x = n[i] 
                y = n[i + 1] 
                if not(i):
                    polygon[0] = [x,y]
                else:
                    polygon.append([x,y])
            i = i + 1
        if not(k):
            vertices_array[0] = polygon
        else:
            vertices_array.append(polygon)
        k = k+1
  
    np.array(vertices_array)
    return vertices_array

# cr https://www.geeksforgeeks.org/find-co-ordinates-of-contours-using-opencv-python/