import cv2 as cv
import numpy as np

# choose a place to save the output frames that's outside of the github repo
# DO NOT COMMIT THE SAVED FRAMES TO THE GITHUB REPO
image_path = r'C:\Users\Quinn\Downloads\training_data\2'
num_im_saved = 0
# open video capture
vid = cv.VideoCapture(0)

# exit the program if video capture isn't working
if not vid.isOpened():
    print("Cannot open camera")
    exit()

# upper and lower values for the yellow hue
lower_yellow = np.array([18, 115, 115])
upper_yellow = np.array([32, 255, 255])

# useful pre-processing step that gets rid of a lot of image noise
def hole_fill(image):
    # blur the image to filter out some of the high frequency noise
    blur = cv.blur(image, (10,10))
    # do a binary mask
    th, bin_mask = cv.threshold(blur, 20, 255, cv.THRESH_BINARY_INV)
    rect_kernel = cv.getStructuringElement(cv.MORPH_RECT, (10, 10))
    # Applying dilation on the threshold image
    dilation = cv.dilate(bin_mask, rect_kernel, iterations=1)

    # Get a bounding box
    # create a contour around the shape
    contours, hierarchy = cv.findContours(dilation, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    # create a bounding box around the largest contour
    if len(contours) != 0:
        c = max(contours, key=cv.contourArea)
        x, y, w, h = cv.boundingRect(c)
        # Crop the image by its bounding box plus ex_pix
        ex_pix = 5
        if y > ex_pix and y + h + ex_pix < dilation.shape[1]:
            y -= ex_pix
            h += ex_pix*2
        if x > ex_pix and x + w + ex_pix < dilation.shape[0]:
            x -= ex_pix
            w += ex_pix*2

        dilation = dilation[y:y + h, x:x + w]

    return dilation

# these variables keep track of the value of a contour capture.
# They go up as more frames are captured in succession
#lvl1 goes up as frames with a capture area of 100 are found
lvl1_capture_val = 0
#lvl2 goes up as frames with a capture area of 625 are found
lvl2_capure_val = 0

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
    training_output = yellow_mask.copy()

    # create a contour around all of the yellow shapes in the image
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
                training_output = hole_fill(training_output)

            # find the point at the center of the bounding box
            xp,yp = (int)(x+w/2), (int)(y+h/2)
            # show a circle representing a moving average of the point
            display_output = cv.circle(display_output, ((int)(xp),(int)(yp)), radius=5, color=(0,0,255), thickness=3)
            # display the bounding box over the output image
            cv.rectangle(display_output, (x, y), (x + w, y + h), (0, 255, 0), 2)

    training_output = hole_fill(training_output)
    # Display the yellow filtered image
    cv.imshow('Yellow Filter', display_output)
    cv.imshow('Training Data', training_output)

    # save the training output if a good crop has been found
    im_width = training_output.shape[0]
    im_height = training_output.shape[1]
    if im_width > 80 and im_width < display_output.shape[0] and im_height > 80 and im_height < display_output.shape[1]:
        # create a file name
        file_name = r"\image2{0}".format(num_im_saved)
        # create a folder path to save the image files in and save the file
        image_folder_path = image_path + r'\image' + file_name + '.png'
        num_im_saved += 1 if cv.imwrite(image_folder_path, training_output) else print("Image wasn't saved")

        # create a folder path to save the text file in and save the file
        text_file = image_path+r'\text'+file_name+r'.gt.txt'
        with open(text_file, 'w') as f:
            f.write('2')

    # press q to stop the program (nothing else will work)
    if cv.waitKey(1) == ord('q'):
        break

# close the video stream
vid.release()
# close the window
cv.destroyAllWindows()