# import libs
import cv2 as cv
import numpy as np
from collections import deque
# A good tutorial on pytesseract here:
# https://nanonets.com/blog/ocr-with-tesseract/#ocr-with-pytesseract-and-opencv
import pytesseract as tes

# Mention the installed location of Tesseract-OCR in your system
#tes.pytesseract.tesseract_cmd = r'C:\Program Files (x86)\Tesseract-OCR\tesseract.exe'
tes.pytesseract.tesseract_cmd = 'C:\Program Files\Tesseract-OCR\\tesseract.exe'

# open video capture
vid = cv.VideoCapture(0)

# exit the program if video capture isn't working
if not vid.isOpened():
    print("Cannot open camera")
    exit()

# upper and lower values for the yellow hue
lower_yellow = np.array([20, 120, 120])
upper_yellow = np.array([30, 255, 255])

avg_pos = deque([])

# calculates a 16 part moving average of an array given a new point
def mvg_avg(avg_pos, new_pos_x, new_pos_y):

    # starting deleting old entries once the size of the list is above 30
    if len(avg_pos) > 15:
        avg_pos.rotate(1)
        avg_pos[0] = (new_pos_x, new_pos_y)
    else:
        avg_pos.append((new_pos_x, new_pos_y))

    # average all of the positions
    xsum = 0
    ysum = 0
    for pos in avg_pos:
        xsum += pos[0]
        ysum += pos[1]
    return xsum/len(avg_pos), ysum/len(avg_pos)

def get_im_text(image):
    # config for image detection
    # Detects only digits
    #custom_config = r'--oem 3 --psm 6 outputbase digits'
    custom_config = r'-c tessedit_char_whitelist=12345 --psm 6'
    # Get a bounding box
    # create a contour around the shape
    contours, hierarchy = cv.findContours(yellow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    # create a bounding box around the largest contour
    c = max(contours, key=cv.contourArea)
    x, y, w, h = cv.boundingRect(c)
    #Crop the image by its bounding box
    if y > 5 and y+h+5 < image.shape[1]:
        y -= 5
        h += 10
    if x > 5 and x+w+5 < image.shape[0]:
        x -= 5
        w += 10

    image = image[y:y+h, x:x+w]

    # Apply OCR on the processed image to find text
    message = tes.image_to_string(image, config=custom_config)


    return message, image

# useful pre-processing step that gets rid of a lot of image noise
def hole_fill(image):
    # blur the image to filter out some of the high frequency noise
    blur = cv.blur(image, (10,10))
    # do a binary mask
    th, bin_mask = cv.threshold(blur, 20, 255, cv.THRESH_BINARY_INV)
    rect_kernel = cv.getStructuringElement(cv.MORPH_RECT, (10, 10))
    # Applying dilation on the threshold image
    dilation = cv.dilate(bin_mask, rect_kernel, iterations=1)

    return dilation

# these variables keep track of the value of a contour capture.
# They go up as more frames are captured in succession
#lvl1 goes up as frames with a capture area of 100 are found
lvl1_capture_val = 0
#lvl2 goes up as frames with a capture area of 625 are found
lvl2_capure_val = 0

# infinite loop
while True:
    # capture frame by frame
    ret, frame = vid.read()

    # make sure a frame is actually returned
    if not ret:
        print("Couldn't receive a frame")
        break

    # convert the frame from rgb to hsv values
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # find all of the yellow in the image
    yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)
    # apply the mask to the display frame
    display_output = cv.bitwise_and(frame, frame, mask=yellow_mask)
    input_text_im = yellow_mask.copy()

    #create a contour around all of the yellow shapes in the image
    contours, hierarchy = cv.findContours(yellow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    # set up bounding box variables
    x, y, w, h = 0, 0, 0, 0
    # make sure at least one contour was found
    text_im = yellow_mask.copy()
    if len(contours) != 0:

        # find the contour with the largest area
        c = max(contours, key = cv.contourArea)
        # only proceed with the processing heavy steps if a sizeable contour was found
        if cv.contourArea(c) > 100 :
            lvl1_capture_val += 1
        else:
            lvl1_capture_val = 0
            lvl2_capure_val = 0

        if lvl1_capture_val > 4:
            # create a bounding box around that contour
            x,y,w,h = cv.boundingRect(c)

            # only try to read the image text if the image is big enough
            if (x+w)*(y+h) > 625:
                lvl2_capure_val += 1
            else:
                lvl2_capure_val = 0

            if lvl2_capure_val > 4:
                # get rid of any noise in the image
                input_text_im = hole_fill(input_text_im)
                # Read the image
                message, text_im = get_im_text(input_text_im)
                # print out the found text
                if message != None and message != "\n" and message != "":
                    print(message)

            # find the point at the center of the bounding box
            xp,yp = (int)(x+w/2), (int)(y+h/2)
            # get a moving average of that point
            x_mvg_avg, y_mvg_avg = mvg_avg(avg_pos, xp, yp)
            # show a circle representing a moving average of the point
            display_output = cv.circle(display_output, ((int)(x_mvg_avg),(int)(y_mvg_avg)), radius=5, color=(0,0,255), thickness=3)
            # display the bounding box over the output image
            cv.rectangle(display_output, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Display the yellow filtered image
    cv.imshow('Yellow Filter', display_output)
    # The hole fill image
    cv.imshow('Hole Filled Output', input_text_im)
    # The image that the text reading is done on
    cv.imshow('Image to Text', text_im)


    # press q to stop the program (nothing else will work)
    if cv.waitKey(1) == ord('q'):
        break

#close the video stream
vid.release()
# close the window
cv.destroyAllWindows()