import cv2
from datetime import datetime
import numpy as np
import yaml
import sys
import traceback

np.set_printoptions(suppress=True, formatter={'float_kind':'{:0.3f}'.format})

# ACHTUNG! Don't run with drive loaded (?)
def main():
    try:
        left_calib_file = "/home/yuchen.wang/Documents/calib/navistar-pdgen2-i003sa_20250313_front_left_camera.yml"
        right_calib_file = "/home/yuchen.wang/Documents/calib/navistar-pdgen2-i003sa_20250313_front_right_camera.yml"

        fs = cv2.FileStorage(left_calib_file, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise FileNotFoundError(f"Unable to open the file: {left_calib_file}")

        height = int(fs.getNode("height").real())
        width = int(fs.getNode("width").real())
        car = fs.getNode("car").string()
        date = fs.getNode("date").string()
        # R, P, M, D, Tr_cam_to_imu
        left_intrinsics = fs.getNode("M").mat()
        left_dist = fs.getNode("D").mat()
        left_pose = fs.getNode("Tr_cam_to_imu").mat()

        fs.release()  # Close the file

        fs = cv2.FileStorage(right_calib_file, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise FileNotFoundError(f"Unable to open the file: {right_calib_file}")

        # Read matrices from the file
        right_intrinsics = fs.getNode("M").mat()
        right_dist = fs.getNode("D").mat()
        right_pose = fs.getNode("Tr_cam_to_imu").mat()

        fs.release()  # Close the file

        print("cam x", left_pose[1, 3], right_pose[1, 3])
        # remember positive y is to the left
        # might need to change the order of the baseline calculation baseline = right_pose[1, 3] - left_pose[1, 3],
        # it should be positive (and probably about 1m)
        baseline = right_pose[1, 3] - left_pose[1, 3]
        tvec = np.array([baseline, 0, 0])

        print(baseline)

        # cv.stereoRectify(	cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T[, R1[, R2[, P1[, P2[, Q[, flags[, alpha[, newImageSize]]]]]]]]	)
        # -> R1, R2, P1, P2, Q, validPixROI1, validPixROI2
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(left_intrinsics, left_dist, right_intrinsics, right_dist, (width, height), np.eye(3), tvec)

        sensor_name = "front_left_right_camera"

        output_filename = f"{car}_{date}_{sensor_name}.yml"
        fs = cv2.FileStorage("data.yml", cv2.FILE_STORAGE_WRITE)

        print(right_intrinsics)

        fs.write("type", "stereo_camera")
        fs.write("car", car)
        fs.write("date", date)
        fs.write("sensor_name", sensor_name)
        fs.write("height", height)
        fs.write("width", width)
        fs.write("rotate", -1)


        # # R, T, D1, R1, R2, D2, M1, M2, P1, P2, Q, Tr_cam_to_imu

        fs.write("R", np.eye(3))
        fs.write("T", tvec)

        fs.write("D1", left_dist)
        fs.write("R1", R1)
        fs.write("D2", right_dist)
        fs.write("R2", R2)

        fs.write("M1", left_intrinsics)
        fs.write("M2", right_intrinsics)
        fs.write("P1", P1)
        fs.write("P2", P2)

        fs.write("Q", Q)
        fs.write("Tr_cam_to_imu", left_pose)

        print(Q)
        print()
    except:
        traceback.print_exc()
        sys.exit("ERROR OCCURED!!!")

if __name__ == '__main__':
    print()
    main()
    print()
    print()
    print("NOW REMEMBER! Go update the front_right config's cam_to_imu and P2!")
    print("Set front_right P to stereo calib P2, and front_right cam_to_imu to front_left cam's.")