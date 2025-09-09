
#!/usr/bin/env python

import sys
import glob
from numpy.core.fromnumeric import around
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ROS Imports to publish transform from base_link to tags
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from tf2_msgs.msg import TFMessage

class TankLeastSquares:
    def __init__(self, tank_id=None, tag_A_samples=None, tag_B_samples=None, tag_C_samples=None):
        self.tank_center = np.eye(4)
        global TANK_NUMBER
        TANK_NUMBER = -1
        if tank_id is None:
            pass
        else:
            TANK_NUMBER = int(tank_id)

        T_A_samples = self.load_samples(tag_B_samples)
        T_B_samples = self.load_samples(tag_C_samples)
        T_C_samples = self.load_samples(tag_A_samples)

        T_A_guess = np.zeros(6)
        T_B_guess = np.zeros(6)
        T_C_guess = np.zeros(6)

        if TANK_NUMBER == 0:
            T_A_guess[0] = 1.259
            T_A_guess[1] = -1.389
            # T_A_guess[2] = 0.163 # - Short WRT Base
            T_A_guess[2] = 0.313 # - Tall WRT Base
            T_A_guess[3] = 0.0 # roll
            T_A_guess[4] = 1.038471 # pitch 59.5 degrees in radians
            T_A_guess[5] = 2.548 #1*(np.pi - 0.610865) # yaw -35.0 degrees in radians

            T_B_guess[0] = -1.136
            T_B_guess[1] = -1.559
            # T_B_guess[2] = 0.163 # - Short WRT Base
            T_B_guess[2] = 0.313 # - Tall WRT Base
            T_B_guess[3] = 0.0 # roll
            T_B_guess[4] = 1.038471 # pitch 59.5 degrees in radians
            T_B_guess[5] = 0.610865 # yaw 35.0 degrees in radians

            T_C_guess[0] = -0.105 # half tag width (210mm) from base_link
            T_C_guess[1] = -0.450 # base_link to tank center tag origin
            T_C_guess[2] = -0.150 # base_link to tank center tag origin
            T_C_guess[3] = 0.0 # roll
            T_C_guess[4] = 0.0 # pitch
            T_C_guess[5] = -1*(np.pi / 2) # yaw -90 degrees in radians

        elif TANK_NUMBER == 1:
            T_A_guess[0] = -1.259
            T_A_guess[1] = 1.389
            # T_A_guess[2] = 0.163 # - Short WRT Base
            T_A_guess[2] = 0.313 # - Tall WRT Base - handeye 0.237
            T_A_guess[3] = 0.0 # roll
            T_A_guess[4] = 1.038471 # pitch 59.5 degrees in radians
            T_A_guess[5] = -1*(0.610865) # yaw -35.0 degrees in radians - handeye -0.597

            T_B_guess[0] = 1.136
            T_B_guess[1] = 1.559
            # T_B_guess[2] = 0.163 # - Short WRT Base
            T_B_guess[2] = 0.313 # - Tall WRT Base - handeye 0.242
            T_B_guess[3] = 0.0 # roll
            T_B_guess[4] = 1.038471 # pitch 59.5 degrees in radians
            T_B_guess[5] = -2.493 #-1*(np.pi - 0.610865) # yaw 35.0 degrees in radians - handeye -2.493

            T_C_guess[0] = 0.105 # half tag width (210mm) from base_link
            T_C_guess[1] = 0.450 # base_link to tank center tag origin
            T_C_guess[2] = -0.150 # base_link to tank center tag origin
            T_C_guess[3] = 0.0 # roll
            T_C_guess[4] = 0.0 # pitch
            T_C_guess[5] = np.pi / 2

        else:
            print(f"Unknown tank number: {TANK_NUMBER}")
            sys.exit(1)

        x0 = np.concatenate((T_A_guess, T_B_guess, T_C_guess))

        result = least_squares(self.objective, x0, args=(T_A_samples, T_B_samples, T_C_samples), verbose=2)

        T_A_vec = result.x[0:6]
        T_B_vec = result.x[6:12]
        T_C_vec = result.x[12:18]

        T_A_result = self.construct_transform_from_euler(T_A_vec)
        T_B_result = self.construct_transform_from_euler(T_B_vec)
        T_C_result = self.construct_transform_from_euler(T_C_vec)


        print(f"T_A_result: {T_A_result}")
        print(f"-- T_A_vector: {self.construct_vector(T_A_result)}")
        print(f"T_B_result: {T_B_result}")
        print(f"-- T_B_vector: {self.construct_vector(T_B_result)}")

        print("\n")
        print(f"-- Translation error: {np.linalg.norm(T_A_vec[:3] - T_B_vec[:3]) - (2.4)}")
        R_diff = R.from_matrix(T_B_result[:3, :3] @ T_A_result[:3, :3].T)
        rot_diff_vector = R_diff.as_rotvec()
        print(f"-- Rotation difference: {rot_diff_vector}")
        #print(f"-- Rotation difference: {np.linalg.norm(T_A_vec[3:] - T_B_vec[3:])}")
        print(f"-- Translation delta: (x,y,z) {T_A_result[:3, 3] - T_B_result[:3, 3]}")
        #print(f"-- Orientation delta: (r,p,y) {T_A_vec[3:] - T_B_vec[3:]}")
        print(f"-- Orientation delta: (r,p,y) {T_A_vec[3:] - T_B_vec[3:]}")
        print("\n")

        print(f"T_C_result: {T_C_result}")

        print(f"-- Distance from origin to T_A_result: {np.linalg.norm(T_A_result[:3, 3])}")
        print(f"-- Distance from origin to T_B_result: {np.linalg.norm(T_B_result[:3, 3])}")
        print(f"-- Distance from origin to T_C_result: {np.linalg.norm(T_C_result[:3, 3])}")

        # Estimate the tank center using geometry
        tank_width = 1.100 # internal including buffers
        tank_center_pos, R_tank = self.estimate_tank_center_with_geometry(T_A_result, T_B_result, T_C_result, tank_width)
        T_tank = np.eye(4)
        T_tank[:3, :3] = R_tank
        T_tank[:3, 3] = tank_center_pos
        print(f"T_tank: {T_tank}")

        self.tank_center = T_tank
        print("Finished - Calibration transform")

    def get_tank_center_pose(self) -> Pose:
        t = Pose()
        T = self.tank_center
        t.position.x = T[0, 3]
        t.position.y = T[1, 3]
        t.position.z = T[2, 3]
        q = R.from_matrix(T[:3, :3]).as_quat()
        t.orientation.x = q[0]
        t.orientation.y = q[1]
        t.orientation.z = q[2]
        t.orientation.w = q[3]
        return t

    def publish_transform(self, T, parent_frame, child_frame):
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = T[2, 3]
        q = R.from_matrix(T[:3, :3]).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)

    def load_samples(self, samples):
        # Effector - tool0
        T_eff = []
        # Object - handeye_trget
        T_obj = []

        for sample in samples:
            T_eff.append(np.array(sample['effector_wrt_world']).reshape(4, 4))
            T_obj.append(np.array(sample['object_wrt_sensor']).reshape(4, 4))

        # NOTE; this is a single calibration result and will ultimatly come
        # from the TF tree with possible refinement
        # Manually build the transformation matrix for the sensor wrt effector
        # tool0 -> camera_color_optical_frame
        T_cam_xyz = np.array([0.040, -0.021, 0.115])
        T_cam_rpy = np.array([0.005, 0.005, -3.137])
        T_cam = np.eye(4)
        T_cam[:3, 3] = T_cam_xyz
        r = R.from_euler('xyz', T_cam_rpy)
        T_cam[:3, :3] = r.as_matrix()

        if len(T_eff) != len(T_obj):
            print("Error: Number of samples do not match")
            sys.exit(1)

        T_ojb_wrt_world = []
        for i in range(len(T_eff)):
            T_ojb_wrt_world.append(T_eff[i] @ T_cam @ T_obj[i])

        return np.array(T_ojb_wrt_world)

    def se3_log(self, T):
        R_mat = T[:3, :3]
        rvec = R.from_matrix(R_mat).as_rotvec()
        tvec = T[:3, 3]
        return np.concatenate((tvec, rvec))

    def construct_transform(self, vector):
        rvec = vector[3:]
        tvec = vector[:3]
        R_mat = R.from_rotvec(rvec).as_matrix()
        T_mat = np.eye(4)
        T_mat[:3, :3] = R_mat
        T_mat[:3, 3] = tvec
        return T_mat

    def construct_transform_from_euler(self, vector):
        r = R.from_euler('xyz', vector[3:], degrees=False)
        R_mat = r.as_matrix()
        tvec = vector[:3]
        T_mat = np.eye(4)
        T_mat[:3, :3] = R_mat
        T_mat[:3, 3] = tvec
        return T_mat

    def construct_vector(self, T):
        R_mat = T[:3, :3]
        rvec = R.from_matrix(R_mat).as_rotvec()
        tvec = T[:3, 3]
        return np.concatenate((tvec, rvec))

    def objective(self, x, T_A_samples, T_B_samples, T_C_samples):
        T_A_vec = x[0:6]
        T_B_vec = x[6:12]
        T_C_vec = x[12:18]

        T_A_guess = self.construct_transform_from_euler(T_A_vec)
        T_B_guess = self.construct_transform_from_euler(T_B_vec)
        T_C_guess = self.construct_transform_from_euler(T_C_vec)

        res = []

        T_A_guess_inv = np.linalg.inv(T_A_guess)
        for sample in T_A_samples:
            res.extend(self.se3_log(T_A_guess_inv @ sample))

        T_B_guess_inv = np.linalg.inv(T_B_guess)
        for sample in T_B_samples:
            res.extend(self.se3_log(T_B_guess_inv @ sample))

        T_C_guess_inv = np.linalg.inv(T_C_guess)
        for sample in T_C_samples:
            res.extend(self.se3_log(T_C_guess_inv @ sample))

        # Apply known constraints
        ####
        ## 1. YAW between Centre and Corners equal and opposite
        ## 2. Distance (XY-Plane) between Centre and Corners
        ##      - TagC to TagA
        ##      - TagC to TagB
        ## 3. Distance between TagA and TagB origins
        ##      - 2400mm
        ## 4. (questionable) Absolute delta Z in Centre tag frame to Corners equal
        ##      - (not measured...) ~30mm
        ####
        ###### Factors
        ## TODO: Develop factors based on the constraints...
        lambda_trans = 100.0
        lambda_rot = 0.5

        ###### 1 ###### - Disabled...
        ## NOTE -- T_A is 'flat' but due to the tank lip orientation other than YAW around Z is unknown
        ## TODO - Rotation around Z (YAW) absolute difference between T_A and T_B is equal to T_A and T_C
        ## 1. YAW between Centre and Corners equal
        ##      - Trying just yaw between B and C being equal to 70 to start with...for now...
        ######
        ## - Rotation between T_A and T_B is 70 degrees
        corner_expected_rot_vec = R.from_euler('z', -110, degrees=True).as_rotvec()
        corner_R_diff = R.from_matrix(T_B_guess[:3, :3] @ T_A_guess[:3, :3].T)
        corner_rot_diff_vector = corner_R_diff.as_rotvec()
        #corner_rot_diff_vector = T_B_vec[3:] - T_A_vec[3:]
        # - NOTE that we want 0 degrees difference, so we don't need to take the norm
        rot_error = (lambda_rot * (corner_expected_rot_vec - corner_rot_diff_vector)).tolist()
        print(f"TB Vector: {T_B_vec}")
        print(f"TB Guess: {T_B_guess}")
        print(f"TA Vector: {T_A_vec}")
        print(f"TA Guess: {T_A_guess}")
        print(f"Rotation error: {rot_error}")
        print(f"Expected rotation vector: {corner_expected_rot_vec}")
        print(f"Rotation difference vector: {corner_rot_diff_vector}")
        #input("Press Enter to continue...")
        # TODO - Is this ready for prime time?
        res.extend(rot_error)
        
        ###### 2 ######
        ###### 3 ###### - kind of same requirement..
        ## NOTE -- T_A is centre flat and T_B / T_C are symetrically placed in empty tile locations
        ## TODO - T_A and T_B are the same distance apart as T_A and T_C 
        ## 2. Distance (XY-Plane) between Centre and Coreners equal
        ##      - 860mm (Y) from Centre side wall & 1200mm (X) from Center
        ######
        ## - Short Tags...
        #tank_ta = [-1.2591, 0.21565, 0.46817] # Taken from CAD (Left)
        #tank_tb = [1.13636, 0.38576, 0.46817] # Taken from CAD (Right)
        #tank_tc = [0.105, -0.72394, 0.305] # Taken from CAD (Center)
        ## - Tall Tags (Estimate...)
        tank_ta = [-1.2591, 0.21565, 0.61817] # Measurement (Left)
        tank_tb = [1.13636, 0.38576, 0.61817] # Measurement (Right)
        tank_tc = [0.105, -0.72394, 0.305] # Taken from CAD (Center)

        #expected_separation_ab_xy = 1.2
        expected_separation_ab = np.linalg.norm(np.array(tank_tb) - np.array(tank_ta))
        ta_tb_separation_error = np.linalg.norm(T_A_vec[:3] - T_B_vec[:3]) - expected_separation_ab
        res.append(lambda_trans * ta_tb_separation_error)

        expected_separation_ac = np.linalg.norm(np.array(tank_tc) - np.array(tank_ta))
        ta_tc_separation_error = np.linalg.norm(T_A_vec[:3] - T_C_vec[:3]) - expected_separation_ac
        res.append(lambda_trans * ta_tc_separation_error)

        expected_separation_bc = np.linalg.norm(np.array(tank_tc) - np.array(tank_tb))
        tb_tc_separation_error = np.linalg.norm(T_B_vec[:3] - T_C_vec[:3]) - expected_separation_bc
        res.append(lambda_trans * tb_tc_separation_error)

        ## pause here for debugging
        #print(f"Expected separation AB: {expected_separation_ab}")
        #print(f"Expected separation AC: {expected_separation_ac}")
        #print(f"Expected separation BC: {expected_separation_bc}")
        #print(f"TA-TB separation error: {ta_tb_separation_error}")
        #print(f"TA-TC separation error: {ta_tc_separation_error}")
        #print(f"TB-TC separation error: {tb_tc_separation_error}")
        #######
        ## readline pause
        #input("Press Enter to continue...")


        ###### 4 ######
        ## TODO...
        ## 4. Absolute delta Z in Centre tag frame to Corners equal
        ##      - (not measured...) ~30mm

        return np.array(res)

    def estimate_tank_center_with_geometry(self, T_A, T_B, T_C, tank_width):
        # Estimate the tank center using calibration points
        # T_A and T_B are 110 degrees rotated around the Z-axis (xy-plane)
        # T_C is mounted in the center of the tank
        # T_A and T_B are mounted at an equal distance from the tank edge (0.265m)
        # Technically we could also use the distance from T_A and T_B from tank ends

        # positions
        pA = T_A[:3, 3]
        pB = T_B[:3, 3]
        pC = T_C[:3, 3]
        
        # Extimated corner tag xz-plane (base_link frame)
        pos_diff = pB - pA
        print(f'Diff = {pos_diff}')
        yawAngle = np.arctan2(pos_diff[1],pos_diff[0])
        print(f'---The yawAngle is...{yawAngle}')

        # Calculate the near mid point between T_A and T_B
        near_mid = (pA + pB) / 2.0

        ## Correct for known X offset to the center tag
        # The only thing we know about the center tag is that it is mounted in the
        # middle of the tank (x-axis)
        tag_offset = 0.105 # half tag width (210mm) as tag origin is in the corner
        if TANK_NUMBER == 0:
            near_mid[0] = pC[0] + tag_offset
        elif TANK_NUMBER == 1:
            near_mid[0] = pC[0] - tag_offset

        ## Correct for known Z offset given both T_A and T_B are equal height
        # - Adjust Z by the known height of TagA and TagB to get tank surface
        # near_mid[2] = near_mid[2] - 0.46817 #- SHORT TAGS
        near_mid[2] = near_mid[2] - 0.61817 #- Estimate of TALL TAGS

        # Calculate the rotation matrix from T_A to T_B and apply the known rotation
        # difference in the Z-axis (xy-plane) - this is the combined rotation from
        # both tags on the base_link frame (actually robot_footprint)
        R_TagA_expected = R.from_euler('xyz', [0, 59.5, -35.0], degrees=True).as_matrix()
        R_TagB_expected = R.from_euler('xyz', [0, 60.0, -142.8], degrees=True).as_matrix()
        R_tank_expected =  R_TagA_expected.T @ R_TagB_expected
        euler_error = R.from_matrix(R_tank_expected).as_euler('xyz', degrees=True)
        print(f"Euler error CAD (degrees): {euler_error}")


        R_tank_predicted = T_A[:3, :3].T @ T_B[:3, :3]
        error = R_tank_predicted @ R_tank_expected.T
        euler_error = R.from_matrix(error).as_euler('xyz', degrees=True)
        print(f"Euler error CAD vs Result (degrees): {euler_error}")
        

        #R_error = R_tank_expected @ R_tank.T
        #R_error_A = R_TagA_expected @ T_A[:3, :3].T
        #R_error_B = R_TagB_expected @ T_B[:3, :3].T
        #euler_error = R.from_matrix(R_error_A).as_euler('xyz', degrees=True)
        #print(f"Euler error (degrees): {euler_error}")
        #euler_error = R.from_matrix(R_error_B).as_euler('xyz', degrees=True)
        #print(f"Euler error (degrees): {euler_error}")
        #input("Press Enter to continue...")
        #R_error = R_error_B

        # NOTE: Assuming 110 is in the CAD...this is a guess...
        #- Rotation: in Quaternion [-0.010, 0.003, 0.041, 0.999]
        #            in RPY (radian) [-0.019, 0.008, 0.082]
        #                        in RPY (degree) [-1.089, 0.440, 4.695]

        #known_rotation = R.from_euler('z', 110.5, degrees=True).as_matrix()
        #known_rotation = R.from_euler('z', 107.5, degrees=True).as_matrix()
        known_rotation = R.from_euler('z', 0.07277, degrees=False).as_matrix()
        current_rotation = R.from_euler('z', yawAngle, degrees=False).as_matrix()
        print(f"Known rotation:\n{known_rotation}")
        print(f"Current rotation:\n{current_rotation}")

        # apply corner_expected_rot_vec to R_tank

        
        #R_tank = T_B[:3, :3] @ T_A[:3, :3].T
        #R_tank = error.T
        R_tank = current_rotation @ known_rotation.T
        #R_tank =  R_tank @ known_rotation
        #R_tank = error.T

        # Assumptions:
        #   1. 0.265 aproximate offset from corner tag posts to inner tank wall
        #   2. Using the y-axis proportion of half the tank width to get center position
        tank_center_pos = np.zeros(3)
        tank_center_pos = near_mid - ((tank_width / 2) - 0.265) * R_tank[:, 1]
        if TANK_NUMBER == 0:
            tank_center_pos = tank_center_pos - (pC[0]+tag_offset) * R_tank[:,0]
            print(f"--pC plus tag_offset would be {(pC[0]+tag_offset)}")
            print(f"--rot trans to apply would be {(pC[0]+tag_offset) * R_tank[:,0]}")
        elif TANK_NUMBER == 1:
            tank_center_pos = tank_center_pos - (pC[0]-tag_offset) * R_tank[:,0]
            print(f"--pC plus tag_offset would be {(pC[0]-tag_offset)}")
            print(f"--rot trans to apply would be {(pC[0]-tag_offset) * R_tank[:,0]}")

        print(f"Tank Center: {tank_center_pos}")

        return tank_center_pos, R_tank
