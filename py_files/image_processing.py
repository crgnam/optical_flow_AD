import cv2
import numpy as np
import csv
import glob
import os


############################
### FUNCTION DEFINITIONS ###
############################
# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))


def read_camera(file_path):
    with open(file_path,newline='') as csvfile:
        camera_details = csv.reader(csvfile)    
        for row in camera_details:
            resolution = [float(row[0]), float(row[1])]
            sensor = [float(row[2]), float(row[3])]
            focal = float(row[4])
    return resolution, sensor, focal


# Define circle least squares fit:
def circle_fit(img_binary):
    pixels = np.squeeze(cv2.findNonZero(img_binary))
    x = pixels[:,0]
    y = pixels[:,1]
    x = x.reshape((-1,1))
    y = y.reshape((-1,1))
    A = np.concatenate((x,y,np.ones(x.shape)),axis=1)
    B = -(x**2 + y**2)
    X = np.squeeze(np.linalg.lstsq(A,B,rcond=None)[0])
    x0 = -0.5*X[0]
    y0 = -0.5*X[1]
    r  =  np.sqrt((X[0]**2 + X[1]**2)/4 - X[2])
    return (x0,y0,r)


# Calculate the roll and pitch of the image:
def get_roll_pitch(img_gray, ang_pix):
    # Threshold Earth in the image:
    _,thresh = cv2.threshold(img_gray,50,255,cv2.THRESH_BINARY)

    # Check if enough of the Earth is visible to continue:
    min_percent = 20
    num_pix = img_gray.shape[0]*img_gray.shape[1]
    num_thresh = cv2.countNonZero(thresh)
    if 100*num_thresh/num_pix < min_percent:
        return None, None, None, None, None
    else:
        # Find and fit the horizon:
        horizon = cv2.Canny(thresh,100,200)
        circle = circle_fit(horizon)
        
        # Calculate pixel nearest to image center point:
        center = np.array([img_gray.shape[1], img_gray.shape[0]])/2

        # Calculate intersection with circle and perpendicular
        circle_center = np.asarray(circle[:2])
        vec = center - circle_center
        vec = vec/np.linalg.norm(vec)
        point = circle_center + circle[2]*vec
        line = [tuple(center.astype('int')), tuple(point.astype('int'))]

        # Calculate pitch and roll angles:
        pos_pitch = thresh[center[1].astype('int'),center[0].astype('int')] == 0
        x_dist = np.abs(center[0] - point[0])
        y_dist = np.abs(center[1] - point[1])
        if pos_pitch:
            roll  = np.arctan([x_dist/y_dist])[0]
            pitch = np.sqrt((x_dist*ang_pix[0])**2 + (y_dist*ang_pix[1])**2)

        else:
            roll  = np.arctan([x_dist/y_dist])[0]
            pitch = np.sqrt((x_dist*ang_pix[0])**2 + (y_dist*ang_pix[1])**2)
        
    return roll, pitch, line, circle, horizon



#################
### MAIN LOOP ###
#################
def main():
    # Need camera details:
    resolution, sensor, focal = read_camera('../data/camera_data.csv')
    ang_pix = [2*np.arctan(sensor[0]/(2*focal)) /resolution[0], 2*np.arctan(sensor[1]/(2*focal))/resolution[1]]

    # Loop through all images:
    old_img = None
    for file_name in glob.glob("../data/*.png"):
        # Read in image:
        img = cv2.imread(file_name)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Calculate roll and pitch via horizon match:
        roll,pitch,line,circle,horizon = get_roll_pitch(img_gray, ang_pix)

        # If a horizon is detected, continue:
        data = None
        if horizon is None:
            old_img = None
        else:
            # Use optical flow to calculate angular rates:
            if old_img is not None:
                p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, img_gray, p0, None, **lk_params)

                # Select good points
                good_new = p1[st==1]
                good_old = p0[st==1]

                # draw the tracks
                for i,(new,old) in enumerate(zip(good_new,good_old)):
                    a,b = new.ravel()
                    c,d = old.ravel()
                    dir_x = a-c
                    dir_y = b-d
                    scale = 5
                    old_img = cv2.line(old_img, (c,d),(int(c + scale*dir_x), int(d + scale*dir_y)), (0,0,255), 2)
                    old_img = cv2.circle(old_img,(c,d),5,(0,0,255),2)
                
                
                # Draw on image:
                old_img = cv2.line(old_img, old_line[0], old_line[1], (0,0,255), 5)
                old_img = cv2.circle(old_img,(int(old_circle[0]),int(old_circle[1])),int(old_circle[2]),(0,0,255),5)

                # Save the output image:
                cv2.imwrite('out_files/{}_detect.png'.format(os.path.splitext(os.path.basename(old_file_name))[0]), old_img)

                # Construct data to be written:
                data = np.array([[roll,pitch]])
                data = np.append(data,good_old,axis=0)
                data = np.append(data,good_new,axis=0)

            
        # Write results to a csv file:
        if data is not None:
            np.savetxt('../data/{}.csv'.format(os.path.splitext(os.path.basename(old_file_name))[0]), data, delimiter=",")

        # Print Processed value:
        print('Processed: ' + file_name)

        # Re-initialize:
        old_file_name = file_name
        old_img = img
        old_gray = img_gray
        old_line = line
        old_circle = circle
        p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)

        
    return 0

if __name__ =="__main__":
    main()