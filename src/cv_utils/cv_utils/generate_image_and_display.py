import numpy as np
import cv2

points_to_circle = []


def click_event(event, x, y, flags, params):
  # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:

        # displaying the coordinates
        # on the Shell
        print(f"clicked on: {x}, {y}")

        global points_to_circle
        points_to_circle.append((x, y))


def main():
    # cap = cv2.VideoCapture(1)  # camera feed is in /dev/video1

    win = cv2.namedWindow("frame")
    cv2.setMouseCallback('frame', click_event)
    while(True):
        image = np.random.rand(480, 640, 3) * 255
        for point in points_to_circle:
            cv2.circle(image, point, 4, (0, 255, 0), -1)

            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow('frame', image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
