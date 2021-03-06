import cv2

cap = cv2.VideoCapture(0)
classifierFace = cv2.CascadeClassifier('/home/gaetan/visionPython/classifier_training/ws/data/cascade.xml')
#classifierFace = cv2.CascadeClassifier('/home/gaetan/catkin_ws/src/LARM-Olive/scripts vision/bottle.xml')

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = classifierFace.detectMultiScale(gray, 1.2, 5)

    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x,y), (x+w, y+h), (255,0,0), 2)
        roiGray = gray[y:y+h, x:x+w]
        roiColor = frame[y:y+h, x:x+w]
    
    cv2.imshow('bot view', gray)
    cv2.imshow('frame', frame)
    
    
    if cv2.waitKey(1)&0xFF==ord('q'):
        break

cap.release()
cv2.destroyAllWindows()