import numpy as np
import cv2 as cv
import glob
import re
import datetime


def log_message(message, level="INFO"):
    """Helper function to log messages with color and timestamps."""
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    level_colors = {
        "INFO": "\033[94m",  # Blue
        "WARNING": "\033[93m",  # Yellow
        "SUCCESS": "\033[92m",  # Green
        "ERROR": "\033[91m",  # Red
    }
    reset_color = "\033[0m"
    color = level_colors.get(level.upper(), "\033[94m")  # Default to Blue for INFO
    print(f"{color}[{level.upper()}] {now} - {message}{reset_color}")


def numerical_sort(value):
    """Helper function to extract numbers from a file name for sorting."""
    numbers = re.findall(r"\d+", value)
    return list(map(int, numbers))


class CharucoCalibrator:

    def __init__(
        self,
        chessboard_size=(10, 7),
        frame_size_h=2592,
        frame_size_w=4608,
        f_in_mm=None,
        pixel_size_mm=None,
        square_mm=20,
        marker_mm=15,
        aruco_dict=cv.aruco.DICT_4X4_250,
        debug=False,
    ):
        self.chessboard_size = chessboard_size
        self.frame_size_h = frame_size_h
        self.frame_size_w = frame_size_w
        self.square_mm = square_mm
        self.marker_mm = marker_mm
        self.aruco_dict = aruco_dict
        self.debug = debug

        # termination criteria
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Intrinsic parameters
        self.f_in_mm = f_in_mm
        self.pixel_size_mm = pixel_size_mm

        if self.f_in_mm is not None and self.pixel_size_mm is not None:
            f_in_pixels = f_in_mm / pixel_size_mm
            cx_in_pixel = (frame_size_w - 1) / 2
            cy_in_pixel = (frame_size_h - 1) / 2

            # Note: if sensor pixel is not square, it needs fx and fy.
            self.known_camera_matrix = np.array(
                [
                    [f_in_pixels, 0, cx_in_pixel],
                    [0, f_in_pixels, cy_in_pixel],
                    [0, 0, 1],
                ],
                dtype=np.float64,
            )
        else:
            self.known_camera_matrix = None

    def calibrate_camera(self, objpoints, imgpoints):
        """Calibrate the camera using the provided image points."""
        if self.known_camera_matrix is not None:
            given_camera_matrix = self.known_camera_matrix.copy()

            ret, camera_matrix, dist, rvecs, tvecs = cv.calibrateCamera(
                objpoints,
                imgpoints,
                (self.frame_size_w, self.frame_size_h),
                given_camera_matrix,  # this matrix will be updated in OpenCV
                distCoeffs=None,
                flags=(
                    cv.CALIB_USE_INTRINSIC_GUESS
                    + cv.CALIB_FIX_PRINCIPAL_POINT
                    + cv.CALIB_FIX_K3
                ),
            )

        else:
            ret, camera_matrix, dist, rvecs, tvecs = cv.calibrateCamera(
                objpoints,
                imgpoints,
                (self.frame_size_w, self.frame_size_h),
                None,
                None,
            )
        return ret, camera_matrix, dist, rvecs, tvecs

    def process_images(self, image_paths):
        """Process images to find Charuco corners and create calibration data."""
        # Sort images to maintain consistent processing
        image_paths.sort(key=numerical_sort)

        # Lists to store object points and image points from all images
        objpoints = []  # 3d point in real-world space
        imgpoints = []  # 2d points in image plane

        # Parameters for ArUco detection
        aruco_dict = cv.aruco.getPredefinedDictionary(self.aruco_dict)

        self.board = cv.aruco.CharucoBoard(
            self.chessboard_size, self.square_mm, self.marker_mm, aruco_dict
        )

        # Create detector
        detector = cv.aruco.CharucoDetector(self.board)
        arucoparams = cv.aruco.DetectorParameters()
        arucoparams.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
        detector.setDetectorParameters(arucoparams)

        print('#   filename      x    y  weight')
        for img_path in image_paths:
            img = cv.imread(img_path)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            # Check frame size
            if gray.shape != (self.frame_size_h, self.frame_size_w):
                raise ValueError(
                    f"File size and frame size do not match. file: {img_path}"
                )

            # Detect Charuco board
            charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)

            if charuco_ids is not None:
                if len(charuco_ids) >= 49:  # if at least 4 charuco corners are found
                    cv.cornerSubPix(
                        gray, charuco_corners, (17, 17), (-1, -1), self.criteria
                    )
                    obj_points, img_points = self.board.matchImagePoints(
                        charuco_corners, charuco_ids
                    )
                    imgpoints.append(img_points)
                    objpoints.append(obj_points)
                    for corner in img_points:
                        print(f'{img_path} {corner[0][0]} {corner[0][1]} 1.0')

                # Optionally, display detected corners on the image
                if self.debug:
                    cv.aruco.drawDetectedCornersCharuco(
                        img, charuco_corners, charuco_ids
                    )
                    for corner in img_points:
                        print(f'{img_path} {corner[0][0]} {corner[0][1]} 1.0')
                        cv.circle(img, (int(corner[0][0]), int(corner[0][1])), 5, (0, 255, 0), -1)
                    cv.imshow("Charuco Detection", img)
                    cv.waitKey(3000)

        cv.destroyAllWindows()
        return objpoints, imgpoints

    def print_pretty_matrix(self, name, matrix):
        """Utility function to print matrices in a readable format with separators."""
        divider = "=" * 50
        print(f"\n{divider}\n{name.upper()}:\n{divider}\n")
        print(np.array2string(matrix, formatter={"float_kind": lambda x: f"{x:0.4f}"}))
        print(f"\n{divider}")


if __name__ == "__main__":
    # Example usage for single camera calibration

    images_path = "*.jpg"
    image_files = glob.glob(images_path)
    # image_files = ['frame0281.jpg']

    chessboard_size = (8, 8)
    frame_size_h = 1300
    frame_size_w = 1600

    # if below is None, then the algorithm will try to deduce it
    f_in_mm = None #4.74
    pixel_size_mm = None #1.4e-3 * 2  # binning factor

    calibrator = CharucoCalibrator(
        chessboard_size=chessboard_size,
        frame_size_h=frame_size_h,
        frame_size_w=frame_size_w,
        f_in_mm=f_in_mm,
        pixel_size_mm=pixel_size_mm,
        debug=False,
    )

    # log_message("Starting image processing for calibration...", "INFO")
    objpoints, imgpoints = calibrator.process_images(image_files)

    # if objpoints and imgpoints:
    #     log_message("Calibrating the camera...", "INFO")
    #     ret, camera_matrix, dist, rvecs, tvecs = calibrator.calibrate_camera(
    #         objpoints, imgpoints
    #     )
    #     log_message(f"🎥 Camera Calibration RMS Error: {ret:.4f}", "SUCCESS")
    #     calibrator.print_pretty_matrix("Camera Matrix", camera_matrix)
    #     calibrator.print_pretty_matrix("Distortion Coefficients", dist)
    # else:
    #     log_message("No valid Charuco corners found in any images.", "ERROR")

