import cv2
import time

cap = cv2.VideoCapture(0)

count = 0
while True:
    count+=1
    ret, frame = cap.read()

    cv2.imshow('frame%d' %count, frame)
    cv2.imwrite("frame%d.jpg" %count, frame)
    print("frame%d.jpg" %count)

    if cv2.waitKey(1)&0xFF==ord('q'):
        break

    time.sleep(2)

cap.release()
cv2.destroyAllWindows()