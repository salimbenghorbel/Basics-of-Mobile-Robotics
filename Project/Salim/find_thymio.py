import cv2
import matplotlib.pyplot as plt
import numpy as np

distance_two_features = 2 #in cm

# Load the image
image1 = cv2.imread('thymio_star.png')
image2 = cv2.imread('thymio_thunder.png')
image3 = cv2.imread('thymio_warped.png')

# Convert the training image to RGB
training_image_1 = cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)
training_image_2 = cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)
test_image = cv2.cvtColor(image3, cv2.COLOR_BGR2RGB)



# Apply thresholding for feature 1 (star)
t_1 = 25  # tolerance
color_1 = [0x41, 0x57, 0x33]

thresh_test_1 = cv2.inRange(test_image, np.array([c - t_1 for c in color_1]), np.array([c + t_1 for c in color_1]))
mask_test_1 = 255 - thresh_test_1

mask_test_1_rgb = cv2.cvtColor(mask_test_1, cv2.COLOR_GRAY2RGB)
test_1_masked = 255 - cv2.max(test_image, mask_test_1_rgb)
test_gray_1 = cv2.cvtColor(test_1_masked, cv2.COLOR_RGB2GRAY)
plt.figure()
plt.imshow(test_gray_1,cmap = 'gray')


thresh_training_1 = cv2.inRange(training_image_1, np.array([c - t_1 for c in color_1]), np.array([c + t_1 for c in color_1]))
mask_training_1 = 255 - thresh_training_1

mask_training_1_rgb = cv2.cvtColor(mask_training_1, cv2.COLOR_GRAY2RGB)
training_1_masked = 255 - cv2.max(training_image_1, mask_training_1_rgb)
training_gray_1 = cv2.cvtColor(training_1_masked, cv2.COLOR_RGB2GRAY)
plt.figure()
plt.imshow(training_gray_1,cmap = 'gray')

# Apply thresholding for feature 2 (lightning)
t_2 = 25  # tolerance
color_2 = [0x33, 0x5D, 0x19]

thresh_test_2 = cv2.inRange(test_image, np.array([c - t_2 for c in color_2]), np.array([c + t_2 for c in color_2]))
mask_test_2 = 255 - thresh_test_2

mask_test_2_rgb = cv2.cvtColor(mask_test_2, cv2.COLOR_GRAY2RGB)
test_2_masked = 255 - cv2.max(test_image, mask_test_2_rgb)
test_gray_2 = cv2.cvtColor(test_2_masked, cv2.COLOR_RGB2GRAY)
plt.figure()
plt.imshow(test_gray_2,cmap = 'gray')


thresh_training_2 = cv2.inRange(training_image_2, np.array([c - t_2 for c in color_2]), np.array([c + t_2 for c in color_2]))
mask_training_2 = 255 - thresh_training_2

mask_training_2_rgb = cv2.cvtColor(mask_training_2, cv2.COLOR_GRAY2RGB)
training_2_masked = 255 - cv2.max(training_image_2, mask_training_2_rgb)
training_gray_2 = cv2.cvtColor(training_2_masked, cv2.COLOR_RGB2GRAY)
plt.figure()
plt.imshow(training_gray_2,cmap = 'gray')



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
test_keypoints_1, test_descriptor_1 = orb.detectAndCompute(test_gray_1, None)
test_keypoints_2, test_descriptor_2 = orb.detectAndCompute(test_gray_2, None)

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

cv2.drawKeypoints(test_image, test_keypoints_1, keypoints_without_size, color = (0, 255, 0))

cv2.drawKeypoints(test_image, test_keypoints_1, keypoints_with_size, flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Display image with and without keypoints size
fx, plots = plt.subplots(1, 2, figsize=(20,10))

plots[0].set_title("Test keypoints With Size")
plots[0].imshow(keypoints_with_size, cmap='gray')

plots[1].set_title("Test keypoints Without Size")
plots[1].imshow(keypoints_without_size, cmap='gray')

keypoints_without_size = np.copy(test_image)
keypoints_with_size = np.copy(test_image)

cv2.drawKeypoints(test_image, test_keypoints_2, keypoints_without_size, color = (0, 255, 0))

cv2.drawKeypoints(test_image, test_keypoints_2, keypoints_with_size, flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Display image with and without keypoints size
fx, plots = plt.subplots(1, 2, figsize=(20,10))

plots[0].set_title("Test keypoints With Size")
plots[0].imshow(keypoints_with_size, cmap='gray')

plots[1].set_title("Test keypoints Without Size")
plots[1].imshow(keypoints_without_size, cmap='gray')

# Print the number of keypoints detected in the training images
print("Number of Keypoints Detected In The Training Image 1: ", len(train_keypoints_1))
print("Number of Keypoints Detected In The Training Image 2: ", len(train_keypoints_2))

# Print the number of keypoints detected in the query images
print("Number of Keypoints Detected In The Query Image 1: ", len(test_keypoints_1))
print("Number of Keypoints Detected In The Query Image 2: ", len(test_keypoints_2))

# Create a Brute Force Matcher object.
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)

# Perform the matching between the ORB descriptors of the training images and the test image
matches_1 = bf.match(train_descriptor_1, test_descriptor_1)
matches_2 = bf.match(train_descriptor_2, test_descriptor_2)

# The matches with shorter distance are the ones we want.
matches_1 = sorted(matches_1, key = lambda x : x.distance)
matches_2 = sorted(matches_2, key = lambda x : x.distance)

result_1 = cv2.drawMatches(training_image_1, train_keypoints_1, test_gray_1, test_keypoints_2, matches_1, test_gray_1, flags = 2)
result_2 = cv2.drawMatches(training_image_2, train_keypoints_2, test_gray_2, test_keypoints_2, matches_2[:30], test_gray_2, flags = 2)
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
test_keypts_pos_1 = np.reshape([test_keypoints_1[mat.trainIdx].pt for mat in matches_1], (len(matches_1),2))

train_keypts_pos_2 = np.reshape([train_keypoints_2[mat.queryIdx].pt for mat in matches_2],(len(matches_2),2))
test_keypts_pos_2 = np.reshape([test_keypoints_2[mat.trainIdx].pt for mat in matches_2], (len(matches_2),2))

weights_1 = [1/match.distance for match in matches_1]
weights_2 = [1/match.distance for match in matches_2]

mean_keypt_1 = np.average(test_keypts_pos_1, weights=weights_1, axis=0)
mean_keypt_2 = np.average(test_keypts_pos_2, weights=weights_2, axis=0)
orientation_vector = mean_keypt_2 - mean_keypt_1
scaling = np.linalg.norm(orientation_vector)/distance_two_features # pixels/cm --> distance_px = distance_cm * scaling