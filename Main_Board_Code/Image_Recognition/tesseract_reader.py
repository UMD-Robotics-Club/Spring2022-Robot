# TUTORIAL LINK
# https://www.geeksforgeeks.org/text-detection-and-extraction-using-opencv-and-ocr/

import cv2 as cv
import pytesseract
import pytesseract as tes

# Mention the installed location of Tesseract-OCR in your system
tes.pytesseract.tesseract_cmd = 'C:\Program Files\Tesseract-OCR\\tesseract.exe'

# open video capture
vid = cv.VideoCapture(0)

# exit the program if video capture isn't working
if not vid.isOpened():
    print("Cannot open camera")
    exit()

while True:
    # capture frame by frame
    ret, frame = vid.read()

    # make sure a frame is actually returned
    if not ret:
        print(" couldn't recieve a frame")


    # pre process the image
    # start by converting to gray scale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # perform OTSU threshold
    ret, thresh1 = cv.threshold(gray, 0, 255, cv.THRESH_OTSU | cv.THRESH_BINARY_INV)

    # Specify structure shape and kernel size.
    # Kernel size increases or decreases the area
    # of the rectangle to be detected.
    # A smaller value like (10, 10) will detect
    # each word instead of a sentence.
    rect_kernel = cv.getStructuringElement(cv.MORPH_RECT, (10, 10))

    # Applying dilation on the threshold image
    dilation = cv.dilate(thresh1, rect_kernel, iterations=1)

    # Find the contours
    contours, hierarchy = cv.findContours(dilation, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    # Creating a copy of the image
    im2 = frame.copy()

    # Looping through the identified contours
    # Then rectangular part is cropped and passed on
    # to pytesseract for extracting text from it
    # Extracted text is then written into the text file
    for cnt in contours:
        x, y, w, h = cv.boundingRect(cnt)

        # Drawing a rectangle on copied image
        rect = cv.rectangle(im2, (x, y), (x+w, y+h), (0,255,0), 2)

        # Cropping the text block for giving input to OCR
        cropped = im2[y:y+h, x:x+w]

        # Apply OCR on the cropped image
        print(pytesseract.image_to_string(cropped))

    cv.imshow('Frame', im2)

    # press q to stop the program (nothing else will work)
    if cv.waitKey(1) == ord('q'):
        break

