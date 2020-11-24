import cv2
import matplotlib.pyplot as plt
import numpy as np

distance_two_features = 2 #in cm

# Load the image
image1 = cv2.imread('thymio_cropped_3.jpg')
image2 = cv2.imread('thymio_cropped_4.jpg')
image3 = cv2.imread('thymio_warped.png')

# Convert the training image to RGB
training_image_1 = cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)
training_image_2 = cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)
test_image = cv2.cvtColor(image3, cv2.COLOR_BGR2RGB)

# Convert the training image to gray scale
training_gray_1 = cv2.cvtColor(training_image_1, cv2.COLOR_RGB2GRAY)
training_gray_2 = cv2.cvtColor(training_image_2, cv2.COLOR_RGB2GRAY)
test_gray = cv2.cvtColor(test_image, cv2.COLOR_RGB2GRAY)

# Display traning image and testing image
fx, plots = plt.subplots(1, 3, figsize=(30,10))

plots[0].set_title("Training Image 1")
plots[0].imshow(training_image_1)

plots[1].set_title("Training Image 2")
plots[1].imshow(training_image_2)

plots[1].set_title("Testing Image")
plots[1].imshow(test_image)

orb = cv2.ORB_create()

train_keypoints_1, train_descriptor_1 = orb.detectAndCompute(training_gray_1, None)
train_keypoints_2, train_descriptor_2 = orb.detectAndCompute(training_gray_2, None)
test_keypoints, test_descriptor = orb.detectAndCompute(test_gray, None)

keypoints_without_size = np.copy(training_image_1)
keypoints_with_size = np.copy(training_image_1)

cv2.drawKeypoints(training_image_1, train_keypoints_1, keypoints_without_size, color = (0, 255, 0))

cv2.drawKeypoints(training_image_1, train_keypoints_1, keypoints_with_size, flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Display image with and without keypoints size
fx, plots = plt.subplots(1, 2, figsize=(20,10))

plots[0].set_title("Train keypoints With Size 1")
plots[0].imshow(keypoints_with_size, cmap='gray')

plots[1].set_title("Train keypoints Without Size 1")
plots[1].imshow(keypoints_without_size, cmap='gray')

keypoints_without_size = np.copy(training_image_2)
keypoints_with_size = np.copy(training_image_2)

cv2.drawKeypoints(training_image_2, train_keypoints_2, keypoints_without_size, color = (0, 255, 0))

cv2.drawKeypoints(training_image_2, train_keypoints_2, keypoints_with_size, flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Display image with and without keypoints size
fx, plots = plt.subplots(1, 2, figsize=(20,10))

plots[0].set_title("Train keypoints With Size 2")
plots[0].imshow(keypoints_with_size, cmap='gray')

plots[1].set_title("Train keypoints Without Size 2")
plots[1].imshow(keypoints_without_size, cmap='gray')

keypoints_without_size = np.copy(test_image)
keypoints_with_size = np.copy(test_image)

cv2.drawKeypoints(test_image, test_keypoints, keypoints_without_size, color = (0, 255, 0))

cv2.drawKeypoints(test_image, test_keypoints, keypoints_with_size, flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Display image with and without keypoints size
fx, plots = plt.subplots(1, 2, figsize=(20,10))

plots[0].set_title("Test keypoints With Size")
plots[0].imshow(keypoints_with_size, cmap='gray')

plots[1].set_title("Test keypoints Without Size")
plots[1].imshow(keypoints_without_size, cmap='gray')

# Print the number of keypoints detected in the training images
print("Number of Keypoints Detected In The Training Image 1: ", len(train_keypoints_1))
print("Number of Keypoints Detected In The Training Image 2: ", len(train_keypoints_2))

# Print the number of keypoints detected in the query image
print("Number of Keypoints Detected In The Query Image: ", len(test_keypoints))


# Create a Brute Force Matcher object.
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)

# Perform the matching between the ORB descriptors of the training images and the test image
matches_1 = bf.match(train_descriptor_1, test_descriptor)
matches_2 = bf.match(train_descriptor_2, test_descriptor)

# The matches with shorter distance are the ones we want.
matches_1 = sorted(matches_1, key = lambda x : x.distance)
matches_2 = sorted(matches_2, key = lambda x : x.distance)

result_1 = cv2.drawMatches(training_image_1, train_keypoints_1, test_gray, test_keypoints, matches_1, test_gray, flags = 2)
result_2 = cv2.drawMatches(training_image_2, train_keypoints_2, test_gray, test_keypoints, matches_2, test_gray, flags = 2)
# Display the best matching points
plt.figure(figsize=(20,10))
plt.rcParams['figure.figsize'] = [14.0, 7.0]
plt.title('Best Matching Points 1')
plt.imshow(result_1)
plt.show()
# Display the best matching points
plt.figure(figsize=(20,10))
plt.rcParams['figure.figsize'] = [14.0, 7.0]
plt.title('Best Matching Points 2')
plt.imshow(result_2)
plt.show()

# Print total number of matching points between the training and query images
print("\nNumber of Matching Keypoints Between The Training 1 and Query Images: ", len(matches_1))
print("\nNumber of Matching Keypoints Between The Training 2 and Query Images: ", len(matches_2))



train_keypts_pos_1 = np.reshape([train_keypoints_1[mat.queryIdx].pt for mat in matches_1],(len(matches_1),2))
test_keypts_pos_1 = np.reshape([test_keypoints[mat.trainIdx].pt for mat in matches_1], (len(matches_1),2))

train_keypts_pos_2 = np.reshape([train_keypoints_2[mat.queryIdx].pt for mat in matches_2],(len(matches_2),2))
test_keypts_pos_2 = np.reshape([test_keypoints[mat.trainIdx].pt for mat in matches_2], (len(matches_2),2))

weights_1 = [1/match.distance for match in matches_1]
weights_2 = [1/match.distance for match in matches_2]

mean_keypt_1 = np.average(test_keypts_pos_1, weights=weights_1, axis=0)
mean_keypt_2 = np.average(test_keypts_pos_2, weights=weights_2, axis=0)
orientation_vector = mean_keypt_2 - mean_keypt_1
scaling = np.linalg.norm(orientation_vector)/distance_two_features # pixels/cm --> distance_px = distance_cm * scaling