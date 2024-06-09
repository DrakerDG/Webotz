import cv2
import numpy as np

def nothing(x):
    pass

def show_xy(winname, img, x, y):
    # Create a named window in normal mode
    cv2.namedWindow(winname, cv2.WINDOW_NORMAL)

    # Move it to (x,y)
    cv2.moveWindow(winname, x, y)

    # Showing window
    cv2.imshow(winname,img)

# video sample (POV)
video_path = './Arduino_Alvik_4sensors_10m.mp4'

# Video capture
cap = cv2.VideoCapture(video_path)
frame_counter = 0

# Font settings
font = cv2.FONT_HERSHEY_PLAIN
fontScale = 1
color = (0, 255, 0)

while(cap.isOpened()):
    ret, frame = cap.read()

    frame_counter += 1

    # Detecting if the video comes to an end to start again
    if frame_counter == cap.get(cv2.CAP_PROP_FRAME_COUNT):
        frame_counter = 0
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    if ret:
        # Copy the image to draw the polygon
        img0 = frame.copy()

        # Frame height and width
        h = frame.shape[0]
        w = frame.shape[1]

        # Center mark vertical position
        ySet = 108

        # Mask for area of interest
        mask = np.zeros((h, w), dtype=np.uint8)

        # Points of the area of interest
        pts = np.array([[[35, 100], [165, 100], [150, 116], [50, 116]]])  # Arduino Alvik target array

        # Drawing the area of interest polygon on the mask
        cv2.fillPoly(mask, pts, 255)

        # Applying the mask to the frame
        zone = cv2.bitwise_and(frame, frame, mask=mask)

        # Generating the HSV image
        hsv = cv2.cvtColor(zone, cv2.COLOR_BGR2HSV)

        # Dark color reference in HSV
        dark_color = np.array([0, 20, 0])

        # Light color reference in HSV
        bght_color = np.array([179, 255, 255])

        # Kernel filter
        Kernel = np.ones((2, 2), np.uint8)

        #Application of the HSV filter on the mask
        mask0 = cv2.inRange(hsv, dark_color, bght_color)

        # Morphological transformation for noise removal
        mask0 = cv2.morphologyEx(mask0, cv2.MORPH_CLOSE, Kernel)
        mask0 = cv2.morphologyEx(mask0, cv2.MORPH_OPEN, Kernel)

        # Contour search
        cnts0, _ = cv2.findContours(mask0, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        # Drawing the contours in the img0
        cv2.drawContours(img0, cnts0, 0, (0, 255, 255), 1)

        try:
            # Select the largest countour
            largest_contour = max(cnts0, key=cv2.contourArea)

            # Obtaining the contour moments
            largest_contour_center = cv2.moments(largest_contour)

            # Calculating the center at x using m10 and m00
            center_x = int(largest_contour_center['m10'] / largest_contour_center['m00'])

            # Drawing a filled circle (-1)
            cv2.circle(frame, (center_x, ySet), 8, (0, 255, 0), 0)

            # Drawing a vertical line
            cv2.line(frame, (center_x, ySet - 20), (center_x, ySet + 20), (0, 255, 0), 1)

            # Drawing a horizontal line
            cv2.line(frame, (center_x - 20, ySet), (center_x + 20, ySet), (0, 255, 0), 1)

            # Write text 'HSV' in hsv image
            cv2.putText(hsv, 'HSV', (70, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

            # Write text 'mask' in mask0 image
            cv2.putText(mask0, 'mask', (60, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        except:
            xval = 1

        # Showing images with contours
        show_xy('Video 0', img0, 360, 130)
        # Showing the detected center
        show_xy('Video 1', frame, 970, 130)
        # Showing the HSV image
        show_xy('HSV', hsv, 360, 550)
        # Showing the mask processed image
        show_xy('Kernel', mask0, 970, 550)

        # Detecting if any key is pressed and if the ESC key
        k = cv2.waitKey(5)
        if k == 27:
            cv2.destroyAllWindows()
            break

# Capture is released
cap.release()