"""A module to do image processing and recognition on numbers 1-5."""
import cv2 as cv
import numpy as np
from collections import deque
# A good tutorial on pytesseract here:
# https://nanonets.com/blog/ocr-with-tesseract/#ocr-with-pytesseract-and-opencv
import pytesseract as tes
#tes.pytesseract.tesseract_cmd = 'C:\Program Files\Tesseract-OCR\\tesseract.exe'#
tes.pytesseract.tesseract_cmd = '/home/robotics/tesseract-4.1.1/src/api/tesseract'

class Camera:
    """A class to handle the camera and do image recognition."""

    def __init__(self, tesseract_path : str, cam_num : int = 0):
        """Initialize the camera, add the path to the tesseract module, and set the bounds for the yellow color."""
        # open up a video stream
        self.cap = cv.VideoCapture(cam_num)
        # exit the program if video capture isn't working
        if not self.cap.isOpened():
            print("Cannot open camera")
        # keep track of the average center of mass of the yellow shapes
        self.avg_pos = deque([])
        # Mention the installed location of Tesseract-OCR in your system
        #tes.pytesseract.tesseract_cmd = tesseract_path
    
    def get_frame(self) -> np.ndarray:
        """Get the current frame from the camera."""
        ret, frame = self.cap.read()
        # make sure a frame is actually returned
        if not ret:
            print("Couldn't receive a frame")
            return False

        self.frame = frame
        return frame

    def find_yellow(self, frame : np.ndarray) -> np.ndarray:
        """Find the yellow shapes in the image.
        
        If no frame is specified, the function will use the last frame from get_frame().
        """
        # upper and lower values for the yellow hue
        lower_yellow = np.array([20, 80, 80])
        upper_yellow = np.array([30, 255, 255])
        # matlab colors
        #lower_yellow = np.array([33, 100, 90])
        #upper_yellow = np.array([46, 255, 255])
        # convert the frame from rgb to hsv values
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # find all of the yellow in the image
        self.yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)
        # apply the mask to the display frame
        masked = cv.bitwise_and(frame, frame, mask=self.yellow_mask)
        return masked

    def denoise_im(self, image : np.ndarray) -> np.ndarray:
        """Pre-process image to get rid of noise."""
        im_copy = image.copy()
        # blur the image to filter out some of the high frequency noise
        blur = cv.blur(im_copy, (10,10))
        # do a binary mask
        th, bin_mask = cv.threshold(blur, 20, 255, cv.THRESH_BINARY_INV)
        rect_kernel = cv.getStructuringElement(cv.MORPH_RECT, (10, 10))
        # Applying dilation on the threshold image
        dilation = cv.dilate(bin_mask, rect_kernel, iterations=1)

        return dilation

    def crop_image(self, image : np.ndarray):
        """Crop the image to the yellow shape.
        
        get_frame() needs to be called before this in order for this function to work properly
        """
        contours, hierarchy = cv.findContours(self.yellow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        cropped = image.copy()
        if len(contours) > 0:
            # create a bounding box around the largest contour
            c = max(contours, key=cv.contourArea)
            x, y, w, h = cv.boundingRect(c)

            cropped = cropped[y:y+h, x:x+w]
            return cropped, (x, y, w, h)
        else:
            return cropped, (0, 0, 0, 0)

    def get_im_text(self):
        """Get the text from the image.
        
        get_frame() needs to be called before this in order for this function to work properly
        """
        # config for image detection
        # Detects only digits
        #custom_config = r'--oem 3 --psm 6 outputbase digits'
        custom_config = r'-c tessedit_char_whitelist=12345 --psm 10'
        denoised_image = self.denoise_im(self.yellow_mask)
        cropped_image, __ = self.crop_image(denoised_image)
        # Apply OCR on the processed image to find text
        message = tes.image_to_string(cropped_image, config=custom_config)

        return message, cropped_image

    def close_video(self):
        """Close the video stream."""
        self.cap.release()
        cv.destroyAllWindows()
        return
    
    def display_image(self, frame=0):
        """Show the image on screen."""
        cv.imshow('frame', self.frame if frame == 0 else frame)
            # press q to stop the program (nothing else will work)
        if cv.waitKey(1) == ord('q'):
            self.close_video()
        return


