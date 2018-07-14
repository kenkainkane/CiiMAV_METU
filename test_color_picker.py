import cv2

import numpy as np



colors = []



def callback(x):

    pass



def on_mouse_click (event, x, y, flags, frame):

    if event == cv2.EVENT_LBUTTONUP:

        colors.append(frame[y,x].tolist())



def main():
    
    color_range={}

    global colors

    capture = cv2.VideoCapture(0)

    cv2.namedWindow('image')



    ilowH = 0

    ihighH = 179

    ilowS = 0

    ihighS = 255

    ilowV = 0

    ihighV = 255



    cv2.createTrackbar('lowH','image',ilowH,179,callback)

    cv2.createTrackbar('highH','image',ihighH,179,callback)



    cv2.createTrackbar('lowS','image',ilowS,255,callback)

    cv2.createTrackbar('highS','image',ihighS,255,callback)



    cv2.createTrackbar('lowV','image',ilowV,255,callback)

    cv2.createTrackbar('highV','image',ihighV,255,callback)



    while True :

        ret, frame = capture.read()

        r,c,ch = frame.shape

        frame = cv2.resize(frame.copy(),(int(c/2),int(r/2)))



        ilowH = cv2.getTrackbarPos('lowH', 'image')

        ihighH = cv2.getTrackbarPos('highH', 'image')

        ilowS = cv2.getTrackbarPos('lowS', 'image')

        ihighS = cv2.getTrackbarPos('highS', 'image')

        ilowV = cv2.getTrackbarPos('lowV','image')

        ihighV = cv2.getTrackbarPos('highV', 'image')



        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if colors:

            ilowH = min(c[0] for c in colors)

            ilowS = min(c[1] for c in colors)

            ilowV = min(c[2] for c in colors)

            ihighH = max(c[0] for c in colors)

            ihighS = max(c[1] for c in colors)

            ihighV = max(c[2] for c in colors)

    

        lower_hsv = np.array([ilowH, ilowS, ilowV])

        upper_hsv = np.array([ihighH, ihighS, ihighV])

        cv2.setTrackbarPos('lowH', 'image', ilowH)

        cv2.setTrackbarPos('highH', 'image', ihighH)

        cv2.setTrackbarPos('lowS', 'image', ilowS)

        cv2.setTrackbarPos('highS', 'image', ihighS)

        cv2.setTrackbarPos('lowV', 'image', ilowV)

        cv2.setTrackbarPos('highV', 'image', ihighV)

        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        frame = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow('mask', mask)

        cv2.imshow('frame', hsv)

        cv2.setMouseCallback('frame', on_mouse_click, hsv)
    
        print(lower_hsv, upper_hsv)

        H = lower_hsv[0]

        S = lower_hsv[1]

        V = lower_hsv[2]

        if not H == 0 and not S == 0 and not V==0 :

            color_range.setdefault('H',[])

            color_range.setdefault('S',[])

            color_range.setdefault('V',[])

            color_range['H'].append(H)

            color_range['S'].append(S)

            color_range['V'].append(V)

            ilowH=min(color_range['H'])

            ihighH=max(color_range['H'])

            ilowS=min(color_range['S'])

            ihighS=max(color_range['S'])

            ilowV=min(color_range['V'])

            ihighV=max(color_range['V'])

            print(ilowH,ihighH)

            print(ilowS,ihighS)

            print(ilowV,ihighV)

            #print(colors)

        colors=[]

        if cv2.waitKey(1) & 0xFF == ord('q'): #Quit

            break

        

        if cv2.waitKey(1) & 0xFF == ord('c'): #Reset Trackbar

            ilowH = 0

            ilowS = 0

            ilowV = 0

            ihighH = 179

            ihighS = 255

            ihighV = 255

            lower_hsv = np.array([ilowH, ilowS, ilowV])

            upper_hsv = np.array([ihighH, ihighS, ihighV])

            cv2.setTrackbarPos('lowH', 'image', ilowH)

            cv2.setTrackbarPos('highH', 'image', ihighH)

            cv2.setTrackbarPos('lowS', 'image', ilowS)

            cv2.setTrackbarPos('highS', 'image', ihighS)

            cv2.setTrackbarPos('lowV', 'image', ilowV)

            cv2.setTrackbarPos('highV', 'image', ihighV)

            colors=[]

            continue



    capture.release()

    cv2.destroyAllWindows()



if __name__ == "__main__":

    main()