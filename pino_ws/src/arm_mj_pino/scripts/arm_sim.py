#!/usr/bin/env python3
import rospy
import rospkg
import mujoco as mj
from mujoco.glfw import glfw
import pinocchio as pino
from pinocchio import SE3
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import tf.transformations as tftf
from numpy.linalg import norm, solve

class MuJoCoBase():
    def __init__(self, xml_path, urdf_path):
        # For callback functions
        self.button_left = False
        self.button_middle = False
        self.button_right = False
        # self.show_left_panel = False
        # self.show_right_panel = False
        self.lastx = 0
        self.lasty = 0
        self.pause_flag = True

        # MuJoCo data structures
        self.model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
        self.data = mj.MjData(self.model)                # MuJoCo data
        self.cam = mj.MjvCamera()                        # Abstract camera
        self.opt = mj.MjvOption()                        # visualization options

        # motor
        self.num_joints = 6
        self.q = np.zeros(self.num_joints)
        self.dq = np.zeros(self.num_joints)
        self.q_tar = np.zeros(self.num_joints)
        self.kp = (15., 15., 25., 15., 10., 10.)
        self.kd = (2.5, 2.5, 5.0, 0.5, 0.2, 0.2)
        # pinocchio
        self.pino_model = pino.buildModelFromUrdf(urdf_path)
        print('model name: ' + self.pino_model.name)
        self.pino_data = self.pino_model.createData()
        self.pino_q = pino.neutral(self.pino_model)
        self.se3 = SE3(np.eye(3), np.array([0.0, 0.0, 0.0]))
        # self.se3base = SE3(np.eye(3), np.array([0.0, 0.0, 0.0]))
        pino.forwardKinematics(self.pino_model, self.pino_data, self.pino_q)
        # for i in range(0, 7):
        #     print(self.pino_model.names[i])
        #     print(self.pino_data.oMi[i])
        for name, oMi in zip(self.pino_model.names, self.pino_data.oMi):
            print("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat))

        # Set subsciber and publisher
        rospy.Subscriber("/ar_pose", Float64MultiArray, self.pose_callback, queue_size=10)

        # Init GLFW, create window, make OpenGL context current, request v-sync
        glfw.init()
        self.window = glfw.create_window(1200, 900, "Demo", None, None)
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        # initialize visualization data structures
        mj.mjv_defaultCamera(self.cam)
        mj.mjv_defaultOption(self.opt)
        self.scene = mj.MjvScene(self.model, maxgeom=10000)
        self.context = mj.MjrContext(
            self.model, mj.mjtFontScale.mjFONTSCALE_150.value)

        # install GLFW mouse and keyboard callbacks
        glfw.set_key_callback(self.window, self.keyboard)
        glfw.set_cursor_pos_callback(self.window, self.mouse_move)
        glfw.set_mouse_button_callback(self.window, self.mouse_button)
        glfw.set_scroll_callback(self.window, self.scroll)

    def keyboard(self, window, key, scancode, act, mods):
        if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
            mj.mj_resetData(self.model, self.data)
            mj.mj_forward(self.model, self.data)
        elif act ==glfw.PRESS and key== glfw.KEY_SPACE:
            self.pause_flag = not self.pause_flag
            mj.mj_forward(self.model, self.data)
        if act == glfw.PRESS and key == glfw.KEY_ESCAPE:
            glfw.terminate()

    def mouse_button(self, window, button, act, mods):
        # update button state
        self.button_left = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        self.button_middle = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
        self.button_right = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

        # update mouse position
        glfw.get_cursor_pos(window)

    def mouse_move(self, window, xpos, ypos):
        # compute mouse displacement, save
        dx = xpos - self.lastx
        dy = ypos - self.lasty
        self.lastx = xpos
        self.lasty = ypos

        # no buttons down: nothing to do
        if (not self.button_left) and (not self.button_middle) and (not self.button_right):
            return

        # get current window size
        width, height = glfw.get_window_size(window)

        # get shift key state
        PRESS_LEFT_SHIFT = glfw.get_key(
            window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
        PRESS_RIGHT_SHIFT = glfw.get_key(
            window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
        mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

        # determine action based on mouse button
        if self.button_right:
            if mod_shift:
                action = mj.mjtMouse.mjMOUSE_MOVE_H
            else:
                action = mj.mjtMouse.mjMOUSE_MOVE_V
        elif self.button_left:
            if mod_shift:
                action = mj.mjtMouse.mjMOUSE_ROTATE_H
            else:
                action = mj.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mj.mjtMouse.mjMOUSE_ZOOM

        mj.mjv_moveCamera(self.model, action, dx/height,
                          dy/height, self.scene, self.cam)

    def scroll(self, window, xoffset, yoffset):
        action = mj.mjtMouse.mjMOUSE_ZOOM
        mj.mjv_moveCamera(self.model, action, 0.0, -0.05 *
                          yoffset, self.scene, self.cam)

    def simulate(self):
        while not glfw.window_should_close(self.window):
            simstart = self.data.time

            while (self.data.time - simstart < 1.0/60.0):
                stepstart = self.data.time
                self.get_sensor_data()
                mj.mj_step1(self.model, self.data)
                self.apply_force()
                mj.mj_step2(self.model, self.data)
                while (self.data.time - stepstart < 0.00099):
                    pass

            # get framebuffer viewport
            viewport_width, viewport_height = glfw.get_framebuffer_size(
                self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            # Update scene and render
            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                               mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(self.window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()

        glfw.terminate()

    def reset(self):
		# Set camera configuration
        self.cam.azimuth = 60
        self.cam.elevation = -15
        self.cam.distance = 1.5
        self.cam.lookat = np.array([0.0, 0.0, 0.5])

    def pose_callback(self, msg):
        x, y, z = msg.data[0], msg.data[1], msg.data[2]
        roll, pitch, yaw = msg.data[3], msg.data[4], msg.data[5]
        judge = msg.data[6]
        if judge > 0.5:
            rotation = tftf.euler_matrix(roll, pitch, yaw, 'sxyz')[:3, :3]
            x += 0.10
            z += 0.16
            translation = np.array([x, y, z])
            self.se3 = SE3(rotation, translation)
            print(self.se3)
            self.cal_inverse_kinematics()

    def controller(self, msg):
        raise NotImplementedError
    
    def get_sensor_data(self):
        self.q[0] = self.data.sensor('joint1Pos').data.copy()
        self.q[1] = self.data.sensor('joint2Pos').data.copy()
        self.q[2] = self.data.sensor('joint3Pos').data.copy()
        self.q[3] = self.data.sensor('joint4Pos').data.copy()
        self.q[4] = self.data.sensor('joint5Pos').data.copy()
        self.q[5] = self.data.sensor('joint6Pos').data.copy()

        self.dq[0] = self.data.sensor('joint1Vel').data.copy()
        self.dq[1] = self.data.sensor('joint2Vel').data.copy()
        self.dq[2] = self.data.sensor('joint3Vel').data.copy()
        self.dq[3] = self.data.sensor('joint4Vel').data.copy()
        self.dq[4] = self.data.sensor('joint5Vel').data.copy()
        self.dq[5] = self.data.sensor('joint6Vel').data.copy()

    def apply_force(self):
        for i in range(0, 6):
            self.data.ctrl[i] = self.kp[i] * (self.q_tar[i] - self.q[i])  + self.kd[i] * (0 - self.dq[i])

    def cal_inverse_kinematics(self):
        # self.pino_q = self.q.T.copy()
        self.pino_q = pino.neutral(self.pino_model)
        eps = 1e-4
        IT_MAX = 1000
        DT = 1e-1
        damp = 1e-12
        i = 0
        while True:
            pino.forwardKinematics(self.pino_model, self.pino_data, self.pino_q)
            iMd = self.pino_data.oMi[self.num_joints].actInv(self.se3)
            err = pino.log(iMd).vector
            if norm(err) < eps:
                success = True
                break
            if i >= IT_MAX:
                success = False
                break
            J = pino.computeJointJacobian(self.pino_model, self.pino_data, self.pino_q, self.num_joints)
            J = -np.dot(pino.Jlog6(iMd.inverse()), J)
            v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
            self.pino_q = pino.integrate(self.pino_model, self.pino_q, v * DT)
            # if not i % 100:
            #     print(f"{i}: error = {err.T}")
            i += 1
        if success:
            print("Convergence achieved!")
        else:
            print("Warning: the iterative algorithm has not reached convergence to the desired precision")
        self.q_tar = self.pino_q.T.copy()
        print(f"\nresult: {self.pino_q.flatten().tolist()}")
        # print(f"\nfinal error: err.T")

def main():
    #ros init
    rospy.init_node('arm_sim', anonymous=True)
    rospy.loginfo("ROS node initialized successfully")
    #get xml path
    rospack = rospkg.RosPack()
    rospack.list()
    arm_desc_path = rospack.get_path('X5A')
    xml_path = arm_desc_path + "/mjcf/X5A.xml"
    urdf_path = arm_desc_path + "/urdf/X5A.urdf"
    # loop_rate = rospy.Rate(1)
    sim = MuJoCoBase(xml_path, urdf_path)
    # sim.reset()
    sim.simulate()

if __name__ == "__main__":
    main()