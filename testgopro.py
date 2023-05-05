import cv2
from time import time
import socket
from goprocam import GoProCamera, constants

WRITE = False
gpCam = GoProCamera.GoPro()
gpCam.overview()



# print('before photo')
# # gpCam.take_photo(timer=0)
# gpCam.stream('udp://127.0.0.1')
# print('after photo')
# gpCam.listMedia(True)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
t=time()
gpCam.livestream("start")
gpCam.video_settings(res='1080p', fps='30')
gpCam.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.R720)
cap = cv2.VideoCapture("udp://10.5.5.9:8554", cv2.CAP_FFMPEG)

# cap = cv2.VideoCapture("udp://127.0.0.1:10000", cv2.CAP_FFMPEG)
counter = 0
while True:
    # print('running')
    nmat, frame = cap.read()
    if frame is not None:
        # print(type(frame))
        # print(frame)
        cv2.imshow("GoPro OpenCV", frame)
    if WRITE == True:
        cv2.imwrite(str(counter)+".jpg", frame)
        counter += 1
        if counter >= 10:
            break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if time() - t >= 2.5:
        sock.sendto("_GPHD_:0:0:2:0.000000\n".encode(), ("10.5.5.9", 8554))
        t=time()
# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()