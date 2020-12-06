import cv2
from skimage.filters import threshold_local
import numpy as np
import imutils
import matplotlib.pyplot as plt
import math

class vision:
    def __init__(self, camera_ip, plot):
        self.camera_ip = camera_ip
        self.plot = plot
    
    def get_camera_image(self):
        '''
        This method captures an image from webcam feed, saves it to a file and
        returns it.
        Returns
        -------
        frame : image
            Image capture from camera feed.

        '''
        webcam = cv2.VideoCapture(self.camera_ip)
        check, frame = webcam.read()
        cv2.imwrite(filename='media/saved_img.png', img=frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        webcam.release()
        return frame

    def get_obstacle_vertices(self, image_gray, scaling_px_2_m):
        '''
        This method takes as input a grayscale image with polygon obstacles such that
        bright pixels are obstacles and dark obstacle are free and returns the 
        coordinates of obstacle vertices in an array.
        cr https://www.geeksforgeeks.org/find-co-ordinates-of-contours-using-opencv-python/

        Parameters
        ----------
        image_gray : image in grayscale
            Image containing obstacles as bright pixels.
        scaling_px_2_m : float
            Image scaling from pixels to meters.

        Returns
        -------
        vertices_array : array
            Array containing the list of vertices coordinates in meters.

        '''

        image_rgb = cv2.cvtColor(image_gray, cv2.COLOR_GRAY2RGB)

        
        # Converting image to a binary image 
        # ( black and white only image). 
        _, threshold = cv2.threshold(image_gray, 110, 255, cv2.THRESH_BINARY) 
       
        # Detecting contours in image. 
        contours, _= cv2.findContours(threshold, cv2.RETR_TREE, 
                                      cv2.CHAIN_APPROX_SIMPLE) 

        vertices_array = [[0]]
        # Going through every contours found in the image. 
        k = 0
        for cnt in contours : 

            approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True) 
      
            # draws boundary of contours. 
            cv2.drawContours(image_rgb, [approx], 0, (0, 0, 255), 5)  
      
            # Used to flatted the array containing 
            # the co-ordinates of the vertices. 
            n = approx.ravel()    
            i = 0
            polygon = [[0,0]]
            for j in n : 
                if(i % 2 == 0): 
                    x = n[i] * scaling_px_2_m
                    y = n[i + 1] * scaling_px_2_m
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
      
        return vertices_array



    def order_points(self, pts):
        '''
        Initialzie a list of coordinates that will be ordered
        such that the first entry in the list is the top-left,
        the second entry is the top-right, the third is the
        bottom-right, and the fourth is the bottom-left

        Parameters
        ----------
        pts : a (4,2) points array.
            A set of points coordinates to order.

        Returns
        -------
        rect : a (4,2) points array.
            The array containing the same input points but ordered like desired.

        '''
        rect = np.zeros((4, 2), dtype = "float32")
        # the top-left point will have the smallest sum, whereas
        # the bottom-right point will have the largest sum
        s = pts.sum(axis = 1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        # now, compute the difference between the points, the
        # top-right point will have the smallest difference,
        # whereas the bottom-left will have the largest difference
        diff = np.diff(pts, axis = 1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        # return the ordered coordinates
        return rect

    def four_point_transform(self, image, pts):
        '''
        cr: https://medium.com/@evergreenllc2020/building-document-scanner-with-opencv-and-python-2306ee65c3db
        This method takes as input an image and a set of points coordinates. It
        extracts the area enclosed within the points and stretches it to a
        rectangular shape.

        Parameters
        ----------
        image : image
            source image.
        pts : (4,2) array of points.
            Corner points to stretch to.

        Returns
        -------
        warped : image
            Part of the image enclosed inside the corner points and stretch to
            have a rectangular shape.

        '''
        # obtain a consistent order of the points and unpack them
        # individually
        rect = self.order_points(pts)
        (tl, tr, br, bl) = rect
        # compute the width of the new image, which will be the
        # maximum distance between bottom-right and bottom-left
        # x-coordiates or the top-right and top-left x-coordinates
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))
        # compute the height of the new image, which will be the
        # maximum distance between the top-right and bottom-right
        # y-coordinates or the top-left and bottom-left y-coordinates
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))
        # now that we have the dimensions of the new image, construct
        # the set of destination points to obtain a "birds eye view",
        # (i.e. top-down view) of the image, again specifying points
        # in the top-left, top-right, bottom-right, and bottom-left
        # order
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype = "float32")
        # compute the perspective transform matrix and then apply it
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
        # return the warped image
        return warped



    def warp_map(self, image):
        '''
        cr: https://medium.com/@evergreenllc2020/building-document-scanner-with-opencv-and-python-2306ee65c3db
        This image takes as input an image of a map taken from a random angle,
        detects the rectangular shape of the map and returns a rectangular image
        containing only the map.
        
        Parameters
        ----------
        image : image
            Image containing the map taken from a random angle.

        Returns
        -------
        warped : image
            An image of the map with transformed perspective.

        '''
        # get the image and compute the ratio of the old height
        # to the new height, clone it, and resize it
        ratio = image.shape[0] / 500.0
        orig = image.copy()
        image = imutils.resize(image, height = 500)
        
        # convert the image to grayscale, blur it, and find edges
        # in the image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        gray = cv2.GaussianBlur(gray, (15,15), 0)
        edged = cv2.Canny(gray, 75, 200)
        
        #show the original image and the edge detected image
        if self.plot:
            fx, plots = plt.subplots(1, 2, figsize=(20,10))
            plots[0].set_title("Image")
            plots[0].imshow(image, cmap='gray')
            plots[1].set_title("Edged")
            plots[1].imshow(edged, cmap='gray')
        
        # find the contours in the edged image, keeping only the
        # largest ones, and initialize the screen contour
        cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:5]
        # loop over the contours
        for c in cnts:
            # approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            # if our approximated contour has four points, then we
            # can assume that we have found our screen
            if len(approx) == 4:
                screenCnt = approx
                break
        
        # apply the four point transform to obtain a top-down
        # view of the original image
        warped = self.four_point_transform(orig, screenCnt.reshape(4, 2) * ratio)
        
        if self.plot:
            # show the original and scanned images
            fx, plots = plt.subplots(1, 2, figsize=(20,10))
            plots[0].set_title("Original")
            plots[0].imshow(imutils.resize(orig, height = warped.shape[0]))
            plots[1].set_title("Warped")
            plots[1].imshow(warped)
        
        return warped


    def create_obstacle_map(self, warped):
        '''
        This method takes as input an image of the warped map and filters it
        in such a way that it extracts the obstacles and the free parts of the
        map. In the input, obstacles are dark and the map is bright. For the 
        output, the obstacles are white pixels.

        Parameters
        ----------
        warped : image
            Warped image of the map.

        Returns
        -------
        obstacle_map : image
            A black and white image with obstacles as white.

        '''
        
        # look for black pixels only
        color_tolerance = 30
        black_thresh = cv2.inRange(warped, np.array([-color_tolerance, -color_tolerance, -color_tolerance]), np.array([color_tolerance, color_tolerance, color_tolerance]))
        black_mask = 255 -black_thresh 
        black_mask_rgb = cv2.cvtColor(black_mask, cv2.COLOR_GRAY2RGB)
        black_masked_rgb = 255 - cv2.max(warped, black_mask_rgb)
        
        
        
        # remove noise: close then open
        black_masked_gray= cv2.cvtColor(black_masked_rgb, cv2.COLOR_RGB2GRAY)
        k1 = 9
        k2 = 25
        se1 = cv2.getStructuringElement(cv2.MORPH_RECT, (k1,k1))
        se2 = cv2.getStructuringElement(cv2.MORPH_RECT, (k2,k2))
        mask_close = cv2.morphologyEx(black_masked_gray, cv2.MORPH_CLOSE, se1)
        mask_close_open = cv2.morphologyEx(mask_close, cv2.MORPH_OPEN, se2)
        
        mask_open_close = mask_close_open / 255
        obstacle_map = black_masked_gray * mask_open_close
        obstacle_map = obstacle_map.astype(np.uint8)
        
        if self.plot:
            fx, plots = plt.subplots(1, 2, figsize=(20,10))
            plots[0].set_title("Only black objects")
            plots[0].imshow(black_masked_gray, cmap= 'gray')
            plots[1].set_title("Filter noise")
            plots[1].imshow(obstacle_map, cmap= 'gray')
        
        return obstacle_map


    def dilate_obstacle_map(self, obstacle_map, thymio_clearance_px):
        '''
        This method performs morphological dilation to obstacle map such that
        it takes nto account the thymio clearance.

        Parameters
        ----------
        obstacle_map : image
            black and white obstacle image.
        thymio_clearance_px : integer
            Amount of pixels for the obstacles to be dilated by.

        Returns
        -------
        dilated_obstacle_map : image
            Dilated obstacle map.

        '''
        # dilate
        kernel = np.ones((thymio_clearance_px,thymio_clearance_px),np.uint8)
        dilated_obstacle_map = cv2.dilate(obstacle_map,kernel,iterations = 1)
        if self.plot:
            plt.figure()
            plt.imshow(dilated_obstacle_map, cmap='gray')
            plt.title('Dilated obstacle map.')
            
        return dilated_obstacle_map


    def locate_target_px(self, warped, target_color):
        '''
        This method locates the target with a certain color from the warped
        map image.

        Parameters
        ----------
        warped : image
            Image of the warped map.
        target_color : (3,) RGB color array.
            RGB color of the target to locate.

        Returns
        -------
        target_position : (2,) position array.
            Position in pixel of the located target.

        '''
        
        color_tolerance = 50
        # extract pixels only within a certain tolerance of the target color.
        pixels_in_range = cv2.inRange(warped, np.array([c - color_tolerance for c in target_color]), np.array([c + color_tolerance for c in target_color]))
        color_mask = 255 - pixels_in_range
        # apply color mask to source image and make it black and white
        color_mask_rgb = cv2.cvtColor(color_mask, cv2.COLOR_GRAY2RGB)
        target_pixels_rgb = 255 - cv2.max(warped, color_mask_rgb)
        target_pixels_gray = cv2.cvtColor(target_pixels_rgb, cv2.COLOR_RGB2GRAY)
        # target position = mean position of all extracted white pixels.
        target_pixels_coordinates = cv2.findNonZero(target_pixels_gray)
        target_position = np.average(target_pixels_coordinates,axis=0)[0]
        
        if self.plot:
            plt.figure()
            plt.imshow(target_pixels_gray, cmap='gray')
            plt.title('Target pixels')
            
            
        return target_position


    def apply_color_filter(self, image, color, color_tolerance):
        '''
        Applies a color filter to an image and extract the pixels that are within
        a certain tolerance from the desired color.

        Parameters
        ----------
        image : image
            Image to be filtered.
        color : (3,) RGB color array/list.
            RGB color to filter with.
        color_tolerance : float
            Color tolerance.

        Returns
        -------
        filtered_pixels_gray : Image
            A black and white image with white pixels as the one within the color
            range.

        '''
        # Create color mask
        pixels_in_range = cv2.inRange(image, np.array([c - color_tolerance for c in color]), np.array([c + color_tolerance for c in color]))
        color_mask = 255 - pixels_in_range
        color_mask_rgb = cv2.cvtColor(color_mask, cv2.COLOR_GRAY2RGB)
        # Filter input image.
        filtered_pixels_rgb = 255 - cv2.max(image, color_mask_rgb)
        filtered_pixels_gray = cv2.cvtColor(filtered_pixels_rgb, cv2.COLOR_RGB2GRAY)
        
        return filtered_pixels_gray


    def locate_feature_in_map_colorpixels(self, feature_color, map_image):
        '''
        apply color filter to map image and get average position of pixels in 
        color range of feature.

        Parameters
        ----------
        feature_color : (3,) RGB color array/list.
            RGB color of the feature to find.
        map_image : image
            Map image.

        Returns
        -------
        feature_position : (2,) pixel position array
            Position in pixel of the feature in map_image.

        '''
        
        color_tolerance = 30
        
        map_filtered_gray = self.apply_color_filter(map_image, feature_color, color_tolerance)
        
        feature_pixels_coordinates = cv2.findNonZero(map_filtered_gray)
        feature_position = np.average(feature_pixels_coordinates,axis=0)[0]    
        print("From locate feature", feature_position)
        if self.plot:
            plt.figure()
            plt.imshow(map_filtered_gray, cmap='gray')
            plt.title('Feature pixels in map')
            
        return feature_position
            
        
    def plot_key_points(self, image, keypoints):
        '''
        This method plots keypoints in map image.

        Parameters
        ----------
        image : image
            Map image.
        keypoints : list/array.
            List of keypoints coordinates in pixel.

        Returns
        -------
        None.

        '''
        keypoints_without_size = np.copy(image)
        keypoints_with_size = np.copy(image)
        
        cv2.drawKeypoints(image, keypoints, keypoints_without_size, color = (0, 255, 0))
        cv2.drawKeypoints(image, keypoints, keypoints_with_size, flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        fx, plots = plt.subplots(1, 2, figsize=(20,10))

        plots[0].set_title("Keypoints With Size")
        plots[0].imshow(keypoints_with_size, cmap='gray')
        
        plots[1].set_title("Keypoints Without Size")
        plots[1].imshow(keypoints_without_size, cmap='gray')


    def locate_feature_in_map_orb(self, map_image, feature_image, feature_color):
        '''
        cr: https://www.geeksforgeeks.org/feature-matching-using-orb-algorithm-in-python-opencv/
        apply color filter to map image and feature image. 
        Then apply ORB algorithm to get position of key points in map image.
        Get average position of key points.
        Detection is rotation and scaling independent but is color dependent.

        Parameters
        ----------
        map_image : image
            Image of the map.
        feature_image : image
            Image of the feature.
        feature_color : (3,) RGB color array/list
            RGB color of the feature.

        Returns
        -------
        feature_position : (2,) pixel position array.
            Position of the feature detected using ORB.

        '''
        
        color_tolerance = 30
        
        feature_filtered_gray = self.apply_color_filter(feature_image, feature_color, color_tolerance)
        map_filtered_gray = self.apply_color_filter(map_image, feature_color, color_tolerance)
        
        if self.plot:
            fx, plots = plt.subplots(1, 2, figsize=(20,10))
            plots[0].set_title("Feature image with color filter")
            plots[0].imshow(feature_filtered_gray, cmap= 'gray')
            plots[1].set_title("Map image with color filter")
            plots[1].imshow(map_filtered_gray, cmap= 'gray')
            
        orb = cv2.ORB_create()

        feature_keypoints, feature_descriptor = orb.detectAndCompute(feature_filtered_gray, None)
        map_keypoints, map_descriptor = orb.detectAndCompute(map_filtered_gray, None)
        
        if self.plot:
            self.plot_key_points(feature_filtered_gray, feature_keypoints)
            self.plot_key_points(map_filtered_gray, map_keypoints)
            
            # Print the number of keypoints detected in the feature image
            print("Number of Keypoints Detected In The Feautre image: ", len(feature_keypoints))
            # Print the number of keypoints detected in the map image
            print("Number of Keypoints Detected In The Map Image: ", len(map_keypoints))
            
        # Create a Brute Force Matcher object.
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)
        
        # Perform the matching between the ORB descriptors of the feature images and the map image
        matches = bf.match(feature_descriptor, map_descriptor)
        
        # The matches with shorter distance are the ones we want.
        matches = sorted(matches, key = lambda x : x.distance)
        
        # Use only keypoints with considerable distance
        matches = matches[:len(matches)//2]
        
        if self.plot:
            results = cv2.drawMatches(feature_filtered_gray, feature_keypoints, map_filtered_gray, map_keypoints, matches, map_image, flags = 2)
            # Display the best matching points
            plt.figure(figsize=(20,10))
            plt.rcParams['figure.figsize'] = [14.0, 7.0]
            plt.title('Best Matching Points')
            plt.imshow(results)
            plt.show()
            
        # Feature Position is the weighted average of the matched keypoints. 
        # Weight is inverse of match distance.
        feature_keypoints_pos = np.reshape([feature_keypoints[mat.queryIdx].pt for mat in matches],(len(matches),2))
        map_keypoints_pos = np.reshape([map_keypoints[mat.trainIdx].pt for mat in matches], (len(matches),2))
        weights = [1/match.distance for match in matches]
        feature_position = np.average(map_keypoints_pos, weights=weights, axis=0)
        
        return feature_position

        

    def locate_thymio(self, map_image, feature_image_front, feature_color_front, feature_image_back, feature_color_back, scaling_px_2_m):
        '''
        This method locates the thymio coordinates and orientation in the warped 
        map based of two images of features purposely placed on top of the 
        thymio. Color filtering is used to improve feature detection algorithms.
        Either ORB or locating with colored pixels in filtered map are available
        to be used.

        Parameters
        ----------
        map_image : image
            Warped image of the map.
        feature_image_front : image
            Image of feature placed on front of thymio.
        feature_color_front : (3,) RGB color array/list
            Color array of the front feature.
        feature_image_back : image
            Image of feature placed on back of thymio.
        feature_color_back : (3,) RGB color array/list
            Color array of the back feature.
        scaling_px_2_m : float
            scaling from pixels to meters.

        Returns
        -------
        thymio_x_m : float
            Thymio x position in meters on the map.
        thymio_y_m : float
            Thymio y position in meters on the map.
        thymio_theta_rad : float
            Thymio orientation on the map.

        '''
        # feature 1 is at the front of thymio, feature 2 is at the back
        # feature_position_front = self.locate_feature_in_map_orb(map_image, feature_image_front, feature_color_front)
        # feature_position_back = self.locate_feature_in_map_orb(map_image, feature_image_back, feature_color_back)
        feature_position_front = self.locate_feature_in_map_colorpixels(feature_color_front, map_image)
        feature_position_back = self.locate_feature_in_map_colorpixels(feature_color_back, map_image)

        # scale coordinates in m.
        thymio_x_m =  feature_position_back[0]  * scaling_px_2_m
        thymio_y_m = feature_position_back[1] * scaling_px_2_m
        # extract orientation with atan2.
        orientation_vector = feature_position_front - feature_position_back
        thymio_theta_rad = math.atan2(orientation_vector[1], orientation_vector[0])
        
        cv2.line(map_image, (int(feature_position_front[0]), int(feature_position_front[1])), (int(feature_position_back[0]), int(feature_position_back[1])), (255, 0, 0), 30)
        plt.imshow(map_image)
        print(feature_position_front,feature_position_back)
        return thymio_x_m, thymio_y_m, thymio_theta_rad
    
    def scale_map(self,map_image, map_x, map_y):
        '''
        This method scales a warped map image to match the real dimensions in
        meters of the physical map.
        Parameters
        ----------
        map_image : image
            Image of the warped map.
        map_x : float
            Map x dimension in m.
        map_y : float
            Map y dimension in m.

        Returns
        -------
        scaled_map : image
            Map scaled with physical aspect ratio.
        scaling_px_2_m : float
            map scaling from pixel to m.

        '''
        # height --> up-down , y
        # width --> right-left, x
        (orig_height, orig_width) = map_image.shape[:2]
        new_height = int(orig_width * map_y/map_x)
        orig = map_image.copy()
        scaled_map =  cv2.resize(orig, (orig_width,new_height))
        scaling_px_2_m = map_y/scaled_map.shape[0]
        return scaled_map, scaling_px_2_m
    
    def draw_line(pointA,pointB,img):
        '''
        This method draws a map between two points on img.

        Parameters
        ----------
        pointA : (2,) position array/list
            position of point A in pixels.
        pointB : (2,) position array/list
            position of point B in pixels..
        img : image
            Map image.

        Returns
        -------
        None.

        '''
        cv2.line(img, (int(pointA[0]), int(pointA[1])), (int(pointB[0]), int(pointB[1])), (255, 0, 0), 30)
        plt.imshow(img)

        
'''
Example of use for the vision module API.
'''
if False:
    map_x = 0.7
    map_y = 1
    thymio_clearance_m = 0.2
    filename = 'media/saved_img.png'
    image = cv2.imread(filename, cv2.IMREAD_COLOR)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    vis = vision(camera_ip=0, plot=True)
    #warped = vis.warp_map(vis.get_camera_image())
    warped = vis.warp_map(image)
    scaled, scaling_px_2_m = vis.scale_map(warped, map_x, map_y)
    # Load the feature images
    feature_image_front = cv2.imread('media/thymio_circle_green.png')
    feature_image_back = cv2.imread('media/thymio_circle_blue.png')
    # Convert the features images to RGB
    feature_image_front = cv2.cvtColor(feature_image_front, cv2.COLOR_BGR2RGB)
    feature_image_back = cv2.cvtColor(feature_image_back, cv2.COLOR_BGR2RGB)
    
    feature_color_front = [8, 72, 47]
    feature_color_back = [39, 52, 104]
    thymio_x_m, thymio_y_m, thymio_theta_rad = vis.locate_thymio(warped, feature_image_front, feature_color_front, feature_image_back, feature_color_back, scaling_px_2_m)
    
    obstacle_map = vis.create_obstacle_map(warped)
    thymio_clearance_px = int(thymio_clearance_m / scaling_px_2_m)
    dilated_obstacle_map = vis.dilate_obstacle_map(obstacle_map, thymio_clearance_px)
    
    target_color = [0x86, 0x33, 0x26]
    target_position_px = vis.locate_target_px(warped, target_color)
    target_position_m = target_position_px * scaling_px_2_m
    
    obstacle_vertices_m = vis.get_obstacle_vertices(dilated_obstacle_map, scaling_px_2_m)
    