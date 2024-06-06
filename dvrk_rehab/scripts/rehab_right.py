#!/usr/bin/env python
from __common_imports__ import *
from functools import update_wrapper, wraps

run_flag_rehab_right = False

class TeleopClass:
    class MODE:
        """
            simulation mode: Use the hardware for the master side, 
                            and simulation for the ECM
            hardware mode: Use hardware for both the master side,
                            and the ECM
        """
        simulation = "SIMULATION"
        hardware = "HARDWARE"
        
    # @param mode hardware or simulation mode
    #  @param self The object pointer.
    def __init__(self, mode = MODE.simulation):
        """!
        Initialize the parameters for the Teleop_class
        @param mode : hardwre or simulation mode
        """
        
        self.__mode__ = mode
        # self.__run__ = False
        
        ## The scale of movements from MTMs to PSMs
        self.scale = 0.5
        
        self.__enabled__ = False
        self.__clutch_active__ = False
         
        self.__last_mtml_pos__ = None
        self.__last_mtml_rot__ = None
        self.__last_mtmr_pos__ = None
        self.__last_mtmr_rot__ = None
    
        self.__last_good_psm1_transform__ = None
        self.__last_good_psm2_transform__ = None
        
        self.__last_psm2_T__ = None
        self.__last_q_2__ = None
        
        self.__mtml_gripper__ = None
        self.__mtmr_gripper__ = None
        
        self.__mtml_jnt__ = None
        self.__mtmr_jnt__ = None
        
        self.__last_psm1_jnt__ = None
        self.__last_psm2_jnt__ = None
        
        self.__last_ecm_jnt__ = None
        
        self.__first_mtml_pos__ = None
        self.__first_mtmr_pos__ = None
        self.__first_psm1_pos__ = None
        self.__first_psm2_pos__ = None
        
        self.__T_ecm__ = None
        self.__T_mtml_000__ = None
        self.__T_mtmr_000__ = None
        self.__arms_homed__ = False
        self.__paused__ = False
        self.__reenable_teleop__ = False
        
        # A dict that buffers the messages based on a delay value
        self.__delay_buffer__ = {}
        self.__delay__ = 2

        self.__last_mtm_source_joint__ = None

        
    def __init__nodes(self):
        """!
        Initialize the ros publishers and subscribers
        """
        
        self.__mtml_robot__ = URDF.from_parameter_server('/dvrk_mtml/robot_description')
        self.__mtmr_robot__ = URDF.from_parameter_server('/dvrk_mtmr/robot_description')
        self.__psm2_robot__ = URDF.from_parameter_server('/dvrk_psm2/robot_description')
        self.__psm1_robot__ = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        self.__ecm_robot__ = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        
        self.__mtml_kin__ = KDLKinematics(self.__mtml_robot__, self.__mtml_robot__.links[0].name, self.__mtml_robot__.links[-1].name)
        self.__mtmr_kin__ = KDLKinematics(self.__mtmr_robot__, self.__mtmr_robot__.links[0].name, self.__mtmr_robot__.links[-1].name)
        self.__psm1_kin__ = KDLKinematics(self.__psm1_robot__, self.__psm1_robot__.links[0].name, self.__psm1_robot__.links[-1].name)
        self.__psm2_kin__ = KDLKinematics(self.__psm2_robot__, self.__psm2_robot__.links[0].name, self.__psm2_robot__.links[-1].name)
        self.__ecm_kin__ = KDLKinematics(self.__ecm_robot__, self.__ecm_robot__.links[0].name, self.__ecm_robot__.links[-1].name)
        
    
        # Subscribe to MTMs
        self.__sub_mtml__ = None; self.__sub_mtmr__ = None
        if self.__mode__ == self.MODE.simulation:
            self.__sub_mtml__ = rospy.Subscriber('/dvrk_mtml/joint_states', JointState, self.__mtml_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_mtmr__ = rospy.Subscriber('/dvrk_mtmr/joint_states', JointState, self.__mtmr_cb__, queue_size=1, tcp_nodelay=True)
        elif self.__mode__ == self.MODE.hardware:
            self.__sub_mtml__ = rospy.Subscriber('/dvrk/MTML/state_joint_current', JointState, self.__mtml_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_mtmr__ = rospy.Subscriber('/dvrk/MTMR/state_joint_current', JointState, self.__mtmr_cb__, queue_size=1, tcp_nodelay=True)

        # subscribe to head sensor
        self.__sub_headsensor__ = rospy.Subscriber('/dvrk/footpedals/coag', Joy, self.__headsensor_cb__ , queue_size=1, tcp_nodelay=True)
        
        # Subscribe to PSMs
        self.__sub_psm1__ = None; self.__sub_psm2__ = None
        if self.__mode__ == self.MODE.simulation:
            self.__sub_psm1__ = rospy.Subscriber('/dvrk_psm1/joint_states', JointState, self.__psm1_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_psm2__ = rospy.Subscriber('/dvrk_psm2/joint_states', JointState, self.__psm2_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_ecm__ = rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.__ecm_cb__, queue_size=1, tcp_nodelay=True)
        else:
            self.__sub_psm1__ = rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, self.__psm1_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_psm2__ = rospy.Subscriber('/dvrk/PSM2/state_joint_current', JointState, self.__psm2_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_ecm__ = rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.__ecm_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_mtml_gripper__ = rospy.Subscriber('/dvrk/MTML/gripper_position_current', Float32, self.__mtml_gripper_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_mtmr_gripper__ = rospy.Subscriber('/dvrk/MTMR/gripper_position_current', Float32, self.__mtmr_gripper_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_mtml_jnt__ = rospy.Subscriber('/dvrk/MTML/state_joint_current', JointState, self.__mtml_jnt_cb, queue_size=1)
            self.__sub_mtmr_jnt__ = rospy.Subscriber('/dvrk/MTMR/state_joint_current', JointState, self.__mtmr_jnt_cb, queue_size=1)
            
        # MTM repositioning clutch
        self.__sub_clutch__ = rospy.Subscriber('/dvrk/footpedals/clutch', Joy, self.__clutch_cb__, queue_size=1, tcp_nodelay=True)
            
        # Publish to PSMs simulation
        self.__pub_psm1__ = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=1)
        self.__pub_psm2__ = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=1)
        self.__pub_ecm__ = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=1)
        
        # Publish to MTMs simulation
        self.__pub_mtml__ = rospy.Publisher('/dvrk_mtml/joint_states_robot', JointState, queue_size=1)
        self.__pub_mtmr__ = rospy.Publisher('/dvrk_mtmr/joint_states_robot', JointState, queue_size=1)
        
        # Translation and orientation lock
        self.__lock_mtml_psm2_orientation__ = rospy.Publisher('/dvrk/MTML_PSM2/lock_rotation', Bool, queue_size=1, latch=True )
        self.__lock_mtml_psm2_translation__ = rospy.Publisher('/dvrk/MTML_PSM2/lock_translation', Bool, queue_size=1, latch=True )
        
        self.__lock_mtmr_psm1_orientation__ = rospy.Publisher('/dvrk/MTMR_PSM1/lock_rotation', Bool, queue_size=1, latch=True )
        self.__lock_mtmr_psm1_translation__ = rospy.Publisher('/dvrk/MTMR_PSM1/lock_translation', Bool, queue_size=1, latch=True )

        # MTML lock orientation
        self.__pub_lock_mtml_orientation__ = rospy.Publisher('/dvrk/MTML/lock_orientation', Quaternion, latch=True, queue_size = 1)
        self.__pub_lock_mtmr_orientation__ = rospy.Publisher('/dvrk/MTMR/lock_orientation', Quaternion, latch=True, queue_size = 1)
        
        self.__pub_unlock_mtml_orientation__ = rospy.Publisher('/dvrk/MTML/unlock_orientation', Empty, latch=True, queue_size = 1)
        self.__pub_unlock_mtmr_orientation__ = rospy.Publisher('/dvrk/MTMR/unlock_orientation', Empty, latch=True, queue_size = 1)
        
        self.__pub_mtmr_psm1_teleop__ = rospy.Publisher('/dvrk/MTMR_PSM1/set_desired_state', String, latch=True, queue_size=1)
        self.__pub_mtml_psm2_teleop__ = rospy.Publisher('/dvrk/MTML_PSM2/set_desired_state', String, latch=True, queue_size=1)
        
        # Wrist Adjustments
        self.__mtml_wrist_adjustment__ = rospy.Publisher('/dvrk/MTML/run_wrist_adjustment', Empty, latch=True, queue_size=1)
        self.__mtmr_wrist_adjustment__ = rospy.Publisher('/dvrk/MTMR/run_wrist_adjustment', Empty, latch=True, queue_size=1)
        
        # Access psm hardware
        self.__hw_psm1__ = robot('PSM1')
        self.__hw_psm2__ = robot('PSM2')
        
        # Access mtm hardware
        self.__hw_mtml__ = robot('MTML')
        self.__hw_mtmr__ = robot('MTMR')

        # Rehab functions - copy MTMR to MTML - Change to array/list
        self.__mtm_target_joint_pub__ = rospy.Publisher('/rehab/MTML/set_position_joint', Float64MultiArray, queue_size=10)
        

        if self.__mode__ == self.MODE.simulation:
            self.enable_teleop()

    
    
    class decorators:
        @classmethod
        def delay(f, name):
            @wraps(f)        
            def _f(*args, **kwargs):
                print(name)
                 
                return
            return _f
                
    def __delay_queue(self, name, msg):
        """!
            Adds messages to a queue. It returns the oldest message that exceeds the delay time. 
            This is used to create a delayed teleoperation.
            
            @param name: The name of the parameter
            @param msg: The message to be stored
        """
        if name not in self.__delay_buffer__.keys():
            self.__delay_buffer__[name] = []
        t = time.time()
        self.__delay_buffer__[name].append( (t,msg))
        if self.__delay_buffer__[name][-1][0]-self.__delay_buffer__[name][0][0] >= self.__delay__:
            return self.__delay_buffer__[name].pop(0)[1]
        else:
            return None
        
    def rehome(self):
        """!
        Home all the arms again
        """
        self.__arms_homed__ = False
        self.home_arms()
        
    def home_arms(self):
        """!
        Move the psms and mtms to a preferred initial position
        """
        
        if self.__arms_homed__ :
            return
        
        q_psm1 = [0.12544035007602872, 0.2371651265674347, 0.13711766733000003, 0.8391791538250665, -0.12269957678936552, -0.14898520116918784, -0.17461480669754448]
        q_psm2 = [-0.01502071544036667, -0.050506672997428295, 0.14912649789000001, -0.9888977734730169, -0.18391272868428285, -0.05774053780206659, -0.17461480669754453]
        q_mtml = [0.0867019358531589, 0.008250772814637434, 0.1410445179152299, -1.498627346290218, 0.0740159621884837, -0.15691383983958546, 0.00592127680054577, 0.0]
        q_mtmr = [0.13146490673506123, -0.06150289811827064, 0.16527587983749847, 1.5291786517226071, 0.28422129480377745, 0.14211064740188872, 0.05329149120491193, 0.0]
        
        r_psm1 = self.__hw_psm1__.move_joint_list(q_psm1, interpolate=True)
        r_psm2 = self.__hw_psm2__.move_joint_list(q_psm2, interpolate=True)
        r_mtml = self.__hw_mtml__.move_joint_list(q_mtml, interpolate=True)
        r_mtmr = self.__hw_mtmr__.move_joint_list(q_mtmr, interpolate=True)
        
        if r_psm1 * r_psm2 * r_mtml * r_mtmr:
            self.__arms_homed__ = True
                
    def shutdown(self):
        """!
        Unregister all the ros publishers and subscribers
        """
        self.shut_down()
        
    def shut_down(self):
        """!
        Same as shutdown
        """
        
        self.__sub_mtml__.unregister()
        self.__sub_mtmr__.unregister()
        self.__sub_psm1__.unregister()
        self.__sub_psm2__.unregister()
        self.__sub_ecm__.unregister()
        self.__sub_headsensor__.unregister()
        self.__sub_clutch__.unregister()
        self.__sub_mtml_gripper__.unregister()
        self.__sub_mtmr_gripper__.unregister()
        
        self.__pub_psm1__.unregister()
        self.__pub_psm2__.unregister()
        self.__pub_mtml__.unregister()
        self.__pub_mtmr__.unregister()
        self.__pub_lock_mtml_orientation__.unregister()
        self.__pub_lock_mtmr_orientation__.unregister()
        self.__pub_unlock_mtml_orientation__.unregister()
        self.__pub_unlock_mtmr_orientation__.unregister()
        
        self.__hw_mtml__.unregister()
        self.__hw_mtmr__.unregister()
        self.__hw_psm1__.unregister()
        self.__hw_psm2__.unregister()
        
    
    def set_mode(self, mode):
        """!
        Set the operation mode to either simulation or hardware
        @param mode : MODE.simulation or MODE.hardware
        """
        self.__mode__ = mode
        
    def spin(self):
        self.__init__nodes()
        rospy.spin()
        
    def pause(self):
        """!
        Pause the teleoperation
        """
        self.__enabled__ = False
        
    def resume(self):
        """!
        Resume the teleopration
        """
        self.__enabled__ = True
                
    def enable_teleop(self):
        """!
        Enable teleoperation
        """

        self.__enabled__ = True
        
        self.__first_mtml_pos__ = self.__last_mtml_pos__
        self.__first_mtmr_pos__ = self.__last_mtmr_pos__
        self.__first_psm1_pos__, _ = self.__psm1_kin__.FK( self.__last_psm1_jnt__)
        self.__first_psm2_pos__, _ = self.__psm2_kin__.FK( self.__last_psm2_jnt__)
        
        self.__hw_mtml__.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
        self.__hw_mtml__.set_wrench_body_force([0,0,0])
        self.__hw_mtml__.set_gravity_compensation(True)
        
        self.__hw_mtmr__.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
        self.__hw_mtmr__.set_wrench_body_force([0,0,0])
        self.__hw_mtmr__.set_gravity_compensation(True)

        
    def disable_teleop(self):
        """!
        Disable teleoperation
        """
        self.__last_mtml_gripper__ = self.__mtml_gripper__
        self.__last_mtmr_gripper__ = self.__mtmr_gripper__
        
        self.__reenable_teleop__ = False
        self.__enabled__ = False
        self.__hw_mtml__.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
        self.__hw_mtml__.set_gravity_compensation(False)
        
        self.__hw_mtmr__.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
        self.__hw_mtmr__.set_gravity_compensation(False)

    def rotate(self, axis, angle):
        """!
        Returns a rotation matrix and a transformation matrix
            @param axis : 'x','y' or 'z'
            @param angle : In radians
            @return r : a 3x3 rotation matrix
            @return t : a 4x4 transformation matrix
        """
        c = np.cos(angle)
        s = np.sin(angle)
        
        r = np.eye(3)
        
        if axis.lower() == 'x':
            r[1:,1:] = np.matrix( [ [c,-s], [s,c]])
        elif axis.lower() == 'z':
            r[:2,:2] = np.matrix( [ [c,-s], [s,c]])
        elif axis.lower() == 'y':
            r[0,0] = c; r[0,2] = s; r[2,0] = -s; r[2,2] = c;
        t = np.eye(4,4)
        t[0:3, 0:3] = r    
        return r, t

    
    def lock_mtm_orientations(self):
        """!
        Lock the orientations of MTMs
        """
        mtml_pose = self.__mtml_kin__.forward(self.__last_mtml_jnt__)
        mtmr_pose = self.__mtmr_kin__.forward(self.__last_mtmr_jnt__)
        mtml_quat = pose_converter.PoseConv.to_pos_quat(mtml_pose)
        mtmr_quat = pose_converter.PoseConv.to_pos_quat(mtmr_pose)
#              
        ql = Quaternion(); ql.w, ql.x, ql.y, ql.z = mtml_quat[1]
        qr = Quaternion(); qr.w, qr.x, qr.y, qr.z = mtmr_quat[1]
         
        self.__pub_lock_mtml_orientation__.publish(ql)
        self.__pub_lock_mtmr_orientation__.publish(qr)
        
    def __clutch_cb__(self, msg):
        """!
        Handle the repositioning clutch
        """
        if msg.buttons[0] == 1 and self.__enabled__:
            if self.__clutch_active__ == False:
                self.__clutch_active__ = True
                self.disable_teleop()
            
            self.lock_mtm_orientations()
            
        elif self.__clutch_active__ == True:
            self.__clutch_active__ = False
            self.__reenable_teleop__ = False
            self.enable_teleop()
            
            
            
            
    def __headsensor_cb__(self, msg):
        """!
        Callback for the head sensor
        """
        if msg.buttons[0] == 1:
            self.enable_teleop()            
        else:
            self.disable_teleop()
            
    def __ecm_cb__(self, msg):
        """!
        Store the ECM joint angles and 
        end-effector position every time a new message is received
        """
        if self.__mode__ == self.MODE.simulation:
            self.__last_ecm_jnt__ = msg.position[0:2] + msg.position[-2:]
        elif self.__mode__ == self.MODE.hardware:
            self.__last_ecm_jnt__ = msg.position[0:3] + tuple([0])
        
        self.__T_ecm__ = self.__ecm_kin__.forward(self.__last_ecm_jnt__)
        if self.__mode__ == self.MODE.hardware:
            self.__pub_ecm__.publish(msg)
        
          
    def __psm1_cb__(self, msg):
        """!
        Store PSM joint angles, and move the simulation if the hardware is active
        """
        if self.__mode__ == self.MODE.simulation:
            p = msg.position
            msg.position = [p[0], p[1], p[7], p[8], p[9], p[10], p[11]]
            
        msg.name =  ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
        self.__last_psm1_jnt__ = msg.position[0:-1]
        if self.__mode__ == self.MODE.hardware:
            self.__pub_psm1__.publish(msg)
            
            
        
    def __psm2_cb__(self, msg):
        """!
        Store PSM joint angles, and move the simulation if the hardware is active
        """
        
        if self.__mode__ == self.MODE.simulation:
            p = msg.position
            msg.position = [p[0], p[1], p[7], p[8], p[9], p[10], p[11]]
            
        msg.name =  ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
        self.__last_psm2_jnt__ = msg.position[0:-1]
        if self.__mode__ == self.MODE.hardware:
            self.__pub_psm2__.publish(msg)
            
    
    def __mtml_gripper_cb__(self, msg):
        """!
        Record position of the gripper
        """
        self.__mtml_gripper__ = msg.data
        
     
    def __mtmr_gripper_cb__(self, msg):
        self.__mtmr_gripper__ = msg.data
                
    
    def __mtml_jnt_cb(self, msg):
        self.__mtml_jnt__ = msg.position
    
    def __mtmr_jnt_cb(self, msg):
        self.__mtmr_jnt__ = msg.position
    

    def __mtml_cb__(self, msg):
        """!
        The main part of the teleoperation is performed in this
        callback function. 
        """
        #self.__copy_joint_positions_l_to_r__(msg)

        if self.__last_ecm_jnt__ is None: return
        if self.__mode__ == self.MODE.simulation:
            msg.position = msg.position[0:2] + msg.position[3:] 
            msg.name = msg.name[0:2] + msg.name[3:]
        else:
            msg.position = msg.position[0:-1]
            msg.name = msg.name[0:-1]

        
        self.__last_mtml_jnt__ = msg.position
        
        if self.__T_mtml_000__ is None:
            self.__T_mtml_000__ = self.__mtml_kin__.forward(msg.position)

        # These rotations help the robot move better
        _, r_330_y_t = self.rotate('y', -np.pi/6.0)
        _, r_330_z_t = self.rotate('z', -np.pi/6.0) 
        _, r_30_x_t = self.rotate('x', np.pi/6.0)
        
        T_mtm = self.__mtml_kin__.forward(msg.position)
        T = ( self.__T_mtml_000__**-1) * T_mtm
        T_rot = T 
        T =  r_330_z_t * T * r_330_y_t * r_30_x_t
        transform = np.matrix( [ [0,-1,0,0], 
                                [0,0,1,0], 
                                [-1,0,0,0], 
                                [0,0,0,1]])
        
        T = transform * T
        T_rot = transform * T_rot

        
  
        pos = T[0:3,3]
        rot = T_rot[0:3,0:3]
        
        if self.__last_mtml_pos__ is None or self.__clutch_active__:
            self.__first_mtml_pos__ = pos
            self.__last_mtml_pos__ = pos
            self.__last_mtml_rot__ = rot
            return
        
        self.__last_mtml_pos__ = pos
        self.__last_mtml_rot__ = rot
        
        if self.__enabled__ == False: return

        
        delta = pos - self.__first_mtml_pos__
        delta = np.insert(delta, 3,1).transpose()
        p0 = np.insert(self.__first_mtml_pos__, 3,1).reshape(4,1)
        p1 = np.insert(pos, 3,1).transpose().reshape(4,1)
        
        translation = (p1-p0)
        translation = ( self.__T_ecm__ * translation)[0:3]
        
        orientation = self.__T_ecm__[0:3,0:3] * rot

        T = self.__translate_mtml__(translation)
        T = self.__set_orientation_mtml__( orientation, T)
        q = list(self.__last_psm2_jnt__)
        q[5] = 0
        q[4] = 0
        q[3] = 0
        
        new_psm2_angles = self.__psm2_kin__.inverse(T, q)
            
        #if type(new_psm2_angles) == NoneType or self.__mtml_jnt__[6] > 1.5 * np.pi or self.__mtml_jnt__[6] < -1.5 * np.pi:
            #self.__hw_mtml__.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
            #self.__hw_mtml__.set_gravity_compensation(False)
            #self.__pub_unlock_mtml_orientation__.publish()
            #return
        #else:
            #self.__hw_mtml__.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
            #self.__hw_mtml__.set_wrench_body_force([0, 0, 0])
            #self.__hw_mtml__.set_gravity_compensation(True)
            
        self.__last_good_psm2_transform__ = T
        
        if self.__mode__ == self.MODE.hardware:
            gripper = (self.__mtml_gripper__-.4) * 1.4/.6
            if (self.__mtml_gripper__ < -.4):
                gripper = -15.0
            new_psm2_angles = np.append(new_psm2_angles, gripper)
            roll_diff = new_psm2_angles[3] - self.__last_psm2_jnt__[3]
           
            if abs(roll_diff) >= np.pi:
                new_roll_diff = 0.0
                    
                if roll_diff > 0 :
                     while not (roll_diff <= np.pi and roll_diff >= -np.pi):
                         roll_diff -= 2*np.pi
                     new_roll_diff = roll_diff
                elif roll_diff < 0:
                    while not (roll_diff >= -np.pi and roll_diff <= np.pi):
                         roll_diff += 2*np.pi 
                    new_roll_diff = roll_diff
                        
                new_roll = new_roll_diff + self.__last_psm2_jnt__[3]
                temp = list(self.__last_psm2_jnt__)
                temp[3] = new_roll
                new_psm2_angles[3] = new_roll
                
            
        if self.__mode__ == self.MODE.hardware:            
            self.__hw_psm2__.move_joint_list( new_psm2_angles.tolist(), range(0,len(new_psm2_angles)), interpolate=False)
        
        if self.__mode__ == self.MODE.simulation:
            msg = JointState()
            msg.position = new_psm2_angles.tolist()
            msg.name =  ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
            self.__pub_psm2__.publish(msg)
            
        self.__last_psm2_T__ = T
        self.__last_q_2__ = q
        
    def __mtmr_cb__(self, msg):

        self.__copy_joint_positions_r_to_l__(msg)
        
        # Find mtm end effector position and orientation
        if self.__last_ecm_jnt__ is None: return
        
        if self.__mode__ == self.MODE.simulation:
            msg.position = msg.position[0:2] + msg.position[3:] 
            msg.name = msg.name[0:2] + msg.name[3:]
        else:
            msg.position = msg.position[0:-1]
            msg.name = msg.name[0:-1]

        self.__last_mtmr_jnt__ = msg.position
        
        if self.__T_mtmr_000__  is None:
            self.__T_mtmr_000__ = self.__T_mtml_000__
        
        # These rotations help the robot move better
        _, r_330_y_t = self.rotate('y', -np.pi/6.0)
        _, r_330_z_t = self.rotate('z', -np.pi/6.0)
        _, r_330_x_t = self.rotate('x', -np.pi/6.0)
        
        T_mtm = self.__mtmr_kin__.forward(msg.position)
        
        T = ( self.__T_mtmr_000__**-1) * T_mtm 
        T_rot = T
        T = r_330_z_t * T * r_330_y_t * r_330_x_t
        
        transform = np.matrix( [ [0,-1,0,0], 
                                [0,0,1,0], 
                                [-1,0,0,0], 
                                [0,0,0,1]])
        T = transform * T
        T_rot = transform * T_rot
        
        rot = T_rot[0:3,0:3] 
        pos = T[0:3,3]
        
        
        if self.__last_mtmr_pos__ is None  or self.__clutch_active__:
            self.__first_mtmr_pos__ = pos
            self.__last_mtmr_pos__ = pos
            self.__last_mtmr_rot__ = rot
            return
        
        self.__last_mtmr_pos__ = pos
        self.__last_mtmr_rot__ = rot
        
        if self.__enabled__ == False: return
        
        delta = pos - self.__first_mtmr_pos__
        delta = np.insert(delta, 3,1).transpose()
        p0 = np.insert(self.__first_mtmr_pos__, 3,1).reshape(4,1)
        p1 = np.insert(pos, 3,1).transpose().reshape(4,1)
        
        translation = (p1-p0)
        translation = ( self.__T_ecm__ * translation)[0:3]
        
        orientation = self.__T_ecm__[0:3,0:3] * rot
        
        T = self.__translate_mtmr__(translation)
        T = self.__set_orientation_mtmr__(orientation, T)
        
        q = list(self.__last_psm1_jnt__)
        q[5] = 0
        q[4] = 0
        q[3] = 0
        new_psm1_angles = self.__psm1_kin__.inverse(T, q)
            
        #if type(new_psm1_angles) == NoneType or self.__mtmr_jnt__[6] > 1.5 * np.pi or self.__mtmr_jnt__[6] < -1.5 * np.pi:
            #self.__hw_mtmr__.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
            #self.__hw_mtmr__.set_gravity_compensation(False)
            #self.__pub_unlock_mtmr_orientation__.publish()
            #return
        #else:
            #self.__hw_mtmr__.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
            #self.__hw_mtmr__.set_wrench_body_force([0, 0, 0])
            #self.__hw_mtmr__.set_gravity_compensation(True)
        
        
        if self.__mode__ == self.MODE.hardware:
            gripper = (self.__mtmr_gripper__-.4) * 1.2/.4
            if (self.__mtmr_gripper__ < -.4):
                gripper = -17.0
            new_psm1_angles = np.append(new_psm1_angles, gripper)
            roll_diff = new_psm1_angles[3] - self.__last_psm1_jnt__[3]
        if abs(roll_diff) > np.pi:                    
            if roll_diff > 0 :
                     while not (roll_diff <= np.pi and roll_diff >= -np.pi):
                         roll_diff -= 2*np.pi
                     new_roll_diff = roll_diff
            elif roll_diff < 0:
                    while not (roll_diff >= -np.pi and roll_diff <= np.pi):
                         roll_diff += 2*np.pi 
                    new_roll_diff = roll_diff
                        
            new_roll = new_roll_diff + self.__last_psm1_jnt__[3]
            new_psm1_angles[3] = new_roll
                
        #if self.__mode__ == self.MODE.hardware:
            #self.__hw_psm1__.move_joint_list( new_psm1_angles.tolist(), range(0,len(new_psm1_angles)), interpolate=False)
        
        if self.__mode__ == self.MODE.simulation:
            msg = JointState()
            msg.position = new_psm1_angles.tolist()
            msg.name =  ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
            self.__pub_psm1__.publish(msg)
            
        
        
    
    def __align_mtms_to_psms(self):
        """!
        This function should align the orientations of mtms to the psms. This is not completed yet.
        """
        T_psm1 = self.__psm1_kin__.forward(self.__last_psm1_jnt__)
        T_psm2 = self.__psm2_kin__.forward(self.__last_psm2_jnt__)
        
        T_mtml = self.__mtml_kin__.forward(self.__last_mtml_jnt__)
        T_mtmr = self.__mtmr_kin__.forward(self.__last_mtmr_jnt__)
        T_mtml_000 = self.__mtml_kin__.forward([0,0,0,0,0,0,0])
        
        T_mtml[0:3,0:3] = (((self.__T_ecm__ * (T_mtml_000 ** -1)) ** -1) * T_psm2)[0:3,0:3]
        T_mtmr[0:3, 0:3] = T_psm1[0:3, 0:3]
        
        jnt_mtml = self.__mtml_kin__.inverse(T_mtml)
        jnt_mtmr = self.__mtmr_kin__.inverse(T_mtmr)
        print('aligning')
        if self.__mode__ == self.MODE.hardware:
            self.__hw_mtml__.move_joint_list( jnt_mtml.tolist(), range(0, len(jnt_mtml)), interpolate=True)
            self.__hw_mtmr__.move_joint_list( jnt_mtmr.tolist(), range(0, len(jnt_mtmr)), interpolate=True)
            
        
        
    def __translate_mtml__(self, translation, T=None): # translate a psm arm
        if self.__enabled__ == False: return
        
        translation = translation * self.scale
        psm2_pos = self.__first_psm2_pos__
        if T is None:
            T = self.__psm2_kin__.forward(self.__last_psm2_jnt__)
        new_psm2_pos = psm2_pos + translation
        T[0:3, 3] = new_psm2_pos
        return T
    
    def __translate_mtmr__(self, translation, T=None): # translate a psm arm
        if self.__enabled__ == False: return
        
        translation = translation * self.scale
        psm1_pos = self.__first_psm1_pos__ 
        
        if T is None:
            T = self.__psm1_kin__.forward(self.__last_psm1_jnt__)
            
        new_psm1_pos = psm1_pos + translation
        T[0:3, 3] = new_psm1_pos
        return T
        
    
    def __set_orientation_mtml__(self,orientation, T=None): # align a psm arm to mtm
        if self.__enabled__ == False: return
        
        if T is None:
            T = self.__psm2_kin__.forward(self.__last_psm2_jnt__)
        T[0:3,0:3] = orientation
        return T
        
    def __set_orientation_mtmr__(self,orientation, T = None): # align a psm arm to mtm
        if self.__enabled__ == False: return
        if T is None:
            T = self.__psm1_kin__.forward(self.__last_psm1_jnt__)
        T[0:3,0:3] = orientation
        return T   


    def __copy_joint_positions_l_to_r__(self, mtm_target_joint_msg):

            msg = Float64MultiArray()
            q_mtmr =  list(mtm_target_joint_msg.position)
            # Reverse wrist_platform and wrist_roll joints
            q_mtmr[3] = -q_mtmr[3]
            q_mtmr[6] = -q_mtmr[6]

            msg.data = q_mtmr
            # Publish joint states to the target MTM
            self.__mtm_target_joint_pub__.publish(msg)

            # Write values to motors
            r_mtmr = self.__hw_mtmr__.move_joint_list(q_mtmr, interpolate=False)

    def __copy_joint_positions_r_to_l__(self, mtm_target_joint_msg):

        msg = Float64MultiArray()
        q_mtml =  list(mtm_target_joint_msg.position)
        # Reverse wrist_platform and wrist_roll joints
        q_mtml[3] = -q_mtml[3]
        q_mtml[6] = -q_mtml[6]

        msg.data = q_mtml
        # Publish joint states to the target MTM
        self.__mtm_target_joint_pub__.publish(msg)

        # Write values to motors
        r_mtml = self.__hw_mtml__.move_joint_list(q_mtml, interpolate=False)

def rehab_right_cb(msg):
    global run_flag_rehab_right
    print("teleop cb called")
    print(msg)
    run_flag_rehab_right = msg

if __name__ == "__main__":
    print("Running teleop Control - rehab version")
    rospy.init_node('teleop_control')

    rospy.Subscriber('/rehab/right/run', Bool, rehab_right_cb)
    node_handler = TeleopClass()
    __mode__ = node_handler.MODE.hardware
    print('\nRunning {} in {}\n'.format("teleop",__mode__))
    node_handler.set_mode(__mode__)    
    
    while not run_flag_rehab_right:
        print("press home")
        time.sleep(1)
    
    try:
        node_handler.spin()
    except rospy.ROSInterruptException:
        pass

