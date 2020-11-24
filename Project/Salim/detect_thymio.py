import numpy as np 
import cv2 
from matplotlib import pyplot as plt
    
# Read the query image as query_img 
# and traing image This query image 
# is what you need to find in train image 
# Save it in the same directory 
# with the name image.jpg   
query_img = cv2.imread('thymio_cropped.jpg') 
train_img = cv2.imread('thymio_path_cropped.jpg') 
   
# Convert it to grayscale 
query_img_bw = cv2.cvtColor(query_img,cv2.COLOR_BGR2GRAY) 
train_img_bw = cv2.cvtColor(train_img, cv2.COLOR_BGR2GRAY) 

#query_img_bw,g,r = cv2.split(query_img)
#train_img_bw,g,r = cv2.split(train_img)
   
# Initialize the ORB detector algorithm 
orb = cv2.ORB_create() 
   
# Now detect the keypoints and compute 
# the descriptors for the query image 
# and train image 
queryKeypoints, queryDescriptors = orb.detectAndCompute(query_img_bw,None) 
trainKeypoints, trainDescriptors = orb.detectAndCompute(train_img_bw,None) 
  
# Initialize the Matcher for matching 
# the keypoints and then match the 
# keypoints 
matcher = cv2.BFMatcher() 
matches = matcher.match(queryDescriptors,trainDescriptors) 
   
# draw the matches to the final image 
# containing both the images the drawMatches() 
# function takes both images and keypoints 
# and outputs the matched query image with 
# its train image 
final_img = cv2.drawMatches(query_img, queryKeypoints,  
train_img, trainKeypoints, matches[:10],None) 
   
final_img = cv2.resize(final_img, (1000,650)) 
  

final_img = cv2.cvtColor(final_img, cv2.COLOR_BGR2RGB)
# Show the final image 
plt.imshow(final_img)
plt.title('Matches')
plt.show()