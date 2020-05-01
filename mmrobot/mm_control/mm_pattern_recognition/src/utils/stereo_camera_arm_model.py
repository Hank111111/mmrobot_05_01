import cv2
import rospkg
from os.path import join
import numpy as np
class CameraModel:
    def __init__(self):
        self.intrinsic_mat = None
        self.distortion_mat = None
        self.image_size = None
        self.projection_mat = None
        
class StereoCameraArmModel():
    def __init__(self):
        self.left_camera = CameraModel()
        self.right_camera = CameraModel()
        self.right_cam_transform_mat = None # right camera's transform matrix comparing to the origin( usually the left camera )
        self.left_cam_transform_mat = None  # usually identity
        self.endeffector_to_cam_transform = None # endeffector's origin to camera's transform
        self.essential_mat = None
        self.fundamental_mat = None
        self.left_map1 = None
        self.left_map2 = None
        self.right_map1, self.right_map2 = None, None

    def loadParams(self, path):
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        self.left_camera.intrinsic_mat = fs.getNode("left_camera_intrinsic").mat()
        self.left_camera.distortion_mat = fs.getNode("left_camera_distortion").mat()

        self.left_camera.image_size = (int(fs.getNode("left_camera_image_size").at(0).real()),
                                                int(fs.getNode("left_camera_image_size").at(1).real()))
        
        self.left_camera.projection_mat = fs.getNode("left_camera_projection").mat()

        self.right_camera.intrinsic_mat = fs.getNode("right_camera_intrinsic").mat()
        self.right_camera.distortion_mat = fs.getNode("right_camera_distortion").mat()
        self.right_camera.image_size = (int(fs.getNode("right_camera_image_size").at(0).real()),
                                                int(fs.getNode("right_camera_image_size").at(1).real()))
        self.right_camera.projection_mat = fs.getNode("right_camera_projection").mat()

        self.right_cam_transform_mat = fs.getNode("right_cam_transform").mat()
        self.left_cam_transform_mat = fs.getNode("left_cam_transform").mat()

        self.endeffector_to_cam_transform = fs.getNode("endeffector_to_cam_transform").mat()
        self.essential_mat = fs.getNode("essential").mat()
        self.fundamental_mat = fs.getNode("fundamental").mat()
        

        self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(self.left_camera.intrinsic_mat, self.left_camera.distortion_mat, None, \
                                                                     self.left_camera.intrinsic_mat, self.left_camera.image_size, cv2.CV_32FC1)
        self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(self.right_camera.intrinsic_mat, self.right_camera.distortion_mat, None, \
                                                                     self.right_camera.intrinsic_mat, self.right_camera.image_size, cv2.CV_32FC1)
    def loadDefaultParams(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('mm_visual_postion')
        path = join(path, "stereo_cam_arm_model.yaml")
        self.loadParams(path)
    def rectifyLeftImage(self, raw_image):
        return cv2.remap(raw_image, self.left_map1, self.left_map2, cv2.INTER_LINEAR)

    def rectifyRightImage(self, raw_image):
        return cv2.remap(raw_image, self.right_map1, self.right_map2, cv2.INTER_LINEAR)