import math
import eigen
import mc_rbdyn
import mc_control
import mc_rtc
import mc_tasks


class JVRCInterviewController(mc_control.MCPythonController):
    def __init__(self, rm, dt):
        self.qpsolver.addConstraintSet(self.dynamicsConstraint)
        self.qpsolver.addConstraintSet(self.contactConstraint)
        self.qpsolver.addTask(self.postureTask)

        self.addContact(self.robot().name(), "ground", "LeftFoot", "AllGround")
        self.addContact(self.robot().name(), "ground", "RightFoot", "AllGround")

        # Hands: position only
        self.left_pos_task = mc_tasks.PositionTask("l_wrist", self.robots(), 0)
        self.right_pos_task = mc_tasks.PositionTask("r_wrist", self.robots(), 0)

        # Head/neck: yaw and pitch
        self.neck_yaw_task = mc_tasks.OrientationTask("NECK_Y_S", self.robots(), 0)
        self.neck_pitch_task = mc_tasks.OrientationTask("NECK_P_S", self.robots(), 0)

        self.qpsolver.addTask(self.left_pos_task)
        self.qpsolver.addTask(self.right_pos_task)
        self.qpsolver.addTask(self.neck_yaw_task)
        self.qpsolver.addTask(self.neck_pitch_task)

        self.left_init_pose = None
        self.right_init_pose = None
        self.neck_yaw_init_pose = None
        self.neck_pitch_init_pose = None

        self.left_goal_pos = None
        self.right_goal_pos = None

        self.look_left_yaw_rot = None
        self.look_right_yaw_rot = None
        self.look_front_yaw_rot = None

        self.look_down_pitch_rot = None
        self.look_front_pitch_rot = None

        self.state = 0
        self.prev_state = -1
        self.state_names = [
            "LEFT_OUT",
            "LEFT_BACK",
            "RIGHT_OUT",
            "RIGHT_BACK",
            "BOTH_OUT",
            "BOTH_BACK",
        ]
        self.state_iter = 0
        self.state_duration = 700

        self.print_every = 100
        self.iteration = 0

    def body_pose(self, body_name):
        idx = self.robot().bodyIndexByName(body_name)
        return self.robot().mbc.bodyPosW[idx]

    def vec_str(self, v):
        return f"({v.x():.4f}, {v.y():.4f}, {v.z():.4f})"

    def rotz(self, angle):
        q = eigen.Quaterniond(math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0))
        q.normalize()
        return q.toRotationMatrix()

    def roty(self, angle):
        q = eigen.Quaterniond(math.cos(angle / 2.0), 0.0, math.sin(angle / 2.0), 0.0)
        q.normalize()
        return q.toRotationMatrix()

    def reset_callback(self, reset_data):
        self.left_pos_task.reset()
        self.right_pos_task.reset()
        self.neck_yaw_task.reset()
        self.neck_pitch_task.reset()

        self.left_init_pose = self.body_pose("l_wrist")
        self.right_init_pose = self.body_pose("r_wrist")
        self.neck_yaw_init_pose = self.body_pose("NECK_Y_S")
        self.neck_pitch_init_pose = self.body_pose("NECK_P_S")

        # Exact challenge pos.
        self.left_goal_pos = eigen.Vector3d(0.5, 0.25, 1.1)
        self.right_goal_pos = eigen.Vector3d(0.5, -0.25, 1.1)

        # Neck base
        self.look_front_yaw_rot = self.neck_yaw_init_pose.rotation()
        self.look_front_pitch_rot = self.neck_pitch_init_pose.rotation()

        yaw = 0.70
        self.look_left_yaw_rot = self.look_front_yaw_rot * self.rotz(-yaw)
        self.look_right_yaw_rot = self.look_front_yaw_rot * self.rotz(+yaw)

        pitch_down = 0.35
        self.look_down_pitch_rot = self.look_front_pitch_rot * self.roty(-pitch_down)

        self.left_pos_task.position(self.left_init_pose.translation())
        self.right_pos_task.position(self.right_init_pose.translation())
        self.neck_yaw_task.orientation(self.look_front_yaw_rot)
        self.neck_pitch_task.orientation(self.look_front_pitch_rot)

        self.state = 0
        self.prev_state = -1
        self.state_iter = 0
        self.iteration = 0

        print("\n[RESET] Controller reset")
        print(f"[RESET] Left init  : {self.vec_str(self.left_init_pose.translation())}")
        print(f"[RESET] Right init : {self.vec_str(self.right_init_pose.translation())}")
        print(f"[RESET] Left goal  : {self.vec_str(self.left_goal_pos)}")
        print(f"[RESET] Right goal : {self.vec_str(self.right_goal_pos)}")

    def run_callback(self):
        if self.state == 0:  # LEFT_OUT
            self.left_pos_task.position(self.left_goal_pos)
            self.right_pos_task.position(self.right_init_pose.translation())
            self.neck_yaw_task.orientation(self.look_left_yaw_rot)
            self.neck_pitch_task.orientation(self.look_down_pitch_rot)

        elif self.state == 1:  # LEFT_BACK
            self.left_pos_task.position(self.left_init_pose.translation())
            self.right_pos_task.position(self.right_init_pose.translation())
            self.neck_yaw_task.orientation(self.look_left_yaw_rot)
            self.neck_pitch_task.orientation(self.look_down_pitch_rot)

        elif self.state == 2:  # RIGHT_OUT
            self.left_pos_task.position(self.left_init_pose.translation())
            self.right_pos_task.position(self.right_goal_pos)
            self.neck_yaw_task.orientation(self.look_right_yaw_rot)
            self.neck_pitch_task.orientation(self.look_down_pitch_rot)

        elif self.state == 3:  # RIGHT_BACK
            self.left_pos_task.position(self.left_init_pose.translation())
            self.right_pos_task.position(self.right_init_pose.translation())
            self.neck_yaw_task.orientation(self.look_right_yaw_rot)
            self.neck_pitch_task.orientation(self.look_down_pitch_rot)

        elif self.state == 4:  # BOTH_OUT
            self.left_pos_task.position(self.left_goal_pos)
            self.right_pos_task.position(self.right_goal_pos)
            self.neck_yaw_task.orientation(self.look_front_yaw_rot)
            self.neck_pitch_task.orientation(self.look_front_pitch_rot)

        elif self.state == 5:  # BOTH_BACK
            self.left_pos_task.position(self.left_init_pose.translation())
            self.right_pos_task.position(self.right_init_pose.translation())
            self.neck_yaw_task.orientation(self.look_front_yaw_rot)
            self.neck_pitch_task.orientation(self.look_front_pitch_rot)

        if self.state != self.prev_state:
            print(f"\n[STATE] -> {self.state_names[self.state]}")
            self.prev_state = self.state

        if self.iteration % self.print_every == 0:
            l_now = self.body_pose("l_wrist").translation()
            r_now = self.body_pose("r_wrist").translation()
            print(f"[ITER {self.iteration:05d}] state={self.state_names[self.state]}")
            print(f"  left  now  = {self.vec_str(l_now)}")
            print(f"  right now  = {self.vec_str(r_now)}")

        self.iteration += 1
        self.state_iter += 1

        if self.state_iter >= self.state_duration:
            self.state = (self.state + 1) % 6
            self.state_iter = 0

        return True

    @staticmethod
    def create(robot, dt):
        env = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH, "ground")
        return JVRCInterviewController([robot, env], dt)