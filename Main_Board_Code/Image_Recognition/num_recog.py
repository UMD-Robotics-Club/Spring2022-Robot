# import libs
import cv2 as cv
import numpy as np
from collections import deque
# A good tutorial on pytesseract here:
# https://nanonets.com/blog/ocr-with-tesseract/#ocr-with-pytesseract-and-opencv
import pytesseract as tes
#tes.pytesseract.tesseract_cmd = 'C:\Program Files\Tesseract-OCR\\tesseract.exe'

class Camera:
    def __init__(self):
        # open up a video stream
        self.cap = cv.VideoCapture(0)
        # exit the program if video capture isn't working
        if not self.cap.isOpened():
            print("Cannot open camera")
            return False
        # upper and lower values for the yellow hue
        self.lower_yellow = np.array([20, 115, 115])
        self.upper_yellow = np.array([30, 255, 255])
        # keep track of the average center of mass of the yellow shapes
        self.avg_pos = deque([])
        # Mention the installed location of Tesseract-OCR in your system
        tes.pytesseract.tesseract_cmd = 'C:\\Program Files (x86)\\Tesseract-OCR\\tesseract.exe'
    
    def get_frame(self):
        ret, frame = self.cap.read()
        # make sure a frame is actually returned
        if not ret:
            print("Couldn't receive a frame")
            return False

        # convert the frame from rgb to hsv values
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # find all of the yellow in the image
        self.yellow_mask = cv.inRange(hsv, self.lower_yellow, self.upper_yellow)
        # apply the mask to the display frame
        self.frame = cv.bitwise_and(frame, frame, mask=self.yellow_mask)
        return self.frame

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

    def crop_image(self, image):
        contours, hierarchy = cv.findContours(self.yellow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        if len(contours) > 0:
            # create a bounding box around the largest contour
            c = max(contours, key=cv.contourArea)
            x, y, w, h = cv.boundingRect(c)

            image = image[y:y+h, x:x+w]
            return image, (x, y, w, h)
        else:
            return image, (0, 0, 0, 0)

    def get_im_text(image):
        # config for image detection
        # Detects only digits
        #custom_config = r'--oem 3 --psm 6 outputbase digits'
        custom_config = r'-c tessedit_char_whitelist=12345 --psm 6'
        text_image = self.crop_image(image)
        # Apply OCR on the processed image to find text
        message = tes.image_to_string(text_image, config=custom_config)

        return message, image

    def mvg_avg(avg_pos, bounds):
        # calculate the box center given x,y, width, height
        new_pos_x = bounds[0] + bounds[2]/2
        new_pos_y = bounds[1] + bounds[3]/2

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

    def close_video(self):
        self.cap.release()
        cv.destroyAllWindows()
        return
    
    def display_image(self):
        cv.imshow('frame', self.frame)
            # press q to stop the program (nothing else will work)
        if cv.waitKey(1) == ord('q'):
            self.close_video()
        return


