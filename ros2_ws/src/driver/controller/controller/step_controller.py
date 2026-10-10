#!/usr/bin/env python3
# encoding: utf-8
import sys
import os
import time
import yaml
import math
import rclpy
import threading
import itertools
import numpy as np
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.msg import CmdParam 
from rclpy.executors import MultiThreadedExecutor

from controller import build_in_pose
from controller.leg_ik import (
    TRACE_PATH_DEFAULT,
    USE_HEXAPOD_KINEMATICS_DEFAULT,
    HoldWatch,
    SecondLog,
    TraceWriter,
    odometry_increment,
    read_max_step_rad,
    should_publish,
    start_gait5,
    startup_line,
    trace_row,
)
from kinematics import kinematics, config, kinematics_calculate
from kinematics.x_joint_control import JointControl
from servo_controller.action_group_controller import ActionGroupController
from controller.pose_transformer import PoseTransformer, PoseTransformerParams
from controller.move import (
    MovingGenerator,
    MovingParams,
    CmdVelGenerator,
    CmdVelParams,
    FollowGaitGenerator,
    generator_intro,
    generator_running,
    generator_outro,
)
from servo_controller_msgs.msg import ServosPosition


def _is_hunter_gait(params):
    """Return True when this mini-step is the hunter follow walk, gait 5."""
    return isinstance(params, CmdVelParams) and params.gait == 5


class _WalkState:
    """The notebook the leg thread keeps for as long as it runs.

    loop creates one of these before the while, and every wake
    reads that same object. It remembers three things, and nothing
    else. generator_slice is the text passed into the walk
    generator on the next resume, and it chooses which part of
    the prepared foot list to read. generator_outro_order says
    how far an order to play the ending has got, which is what
    keeps a velocity walk from jumping to a new generator on the
    same wake that noticed the period had ended.
    pending_generator_after_outro is the generator copied at the
    moment that order was written. The feet, the body twist, and
    the generator that is playing right now live on StepController.
    """

    def __init__(self):
        # A new thread has not played a step yet, so the first
        # resume asks for the opening section of the prepared
        # list. That section runs from the splice point to the
        # end of the table. A cold start uses it, and so does the
        # first period after a new generator is installed.
        self.generator_slice = generator_intro
        # No ending has been asked for. 'ordered' will mean this
        # wake just asked, and the swap has to wait until the
        # feet of this wake have been sent. 'in_effect' will mean
        # the later wakes play the outro, and the next accepted
        # gait step may start the generator we stored.
        self.generator_outro_order = 'off'
        self.pending_generator_after_outro = None

    def after_accepted_gait_step(self, controller, params):
        """React once the caller has decided this gait step counts.

        The caller already knows the period ended. This method
        does not test that again, and it does not look at the
        blend flag. A legacy walk learns the period ended from
        the yield's last_part, inside the pull, before the servos
        of that wake are commanded. Gait 5 learns it from
        mini_step, and only after the gate has sent the six feet
        that close the period. Calling this any earlier on gait 5
        would start the next target while those feet were still
        short of where the generator asked them to be.

        A counted walk carries MovingParams. The generator is the
        whole performance, so there is no ending section to play
        inside it. This call installs new_moving_generator
        immediately. When that queue is empty the walk is over,
        and the twist the node publishes is set to zero so
        odometry stops claiming the body is still moving.

        A velocity walk carries CmdVelParams, and gait 5 is one
        of those. The generator that is playing has to be allowed
        to finish through the ending of its own list, or the feet
        jump. The first accepted gait step that finds a different
        generator waiting only writes the order. The generator
        that was waiting is copied into
        pending_generator_after_outro at that moment. A Twist
        that arrives while the outro plays can replace
        new_moving_generator, and it leaves the copy alone. The
        wake that writes the order still sends the feet it had
        already pulled, which are the last target of the slice
        that just ended. At the end of that same wake the tick
        moves the order from 'ordered' to 'in_effect'. From the
        next resume on, the generator reads its ending section
        and saves each pose so the next generator can start near
        those feet. The order stays 'in_effect' for that whole
        outro. The next accepted gait step finds the order
        already in effect, installs the stored generator, and
        sets the slice back to the opening section so the new
        generator begins at its own splice.
        """
        if isinstance(params, MovingParams):
            # The new generator, or None when nothing is queued,
            # becomes the one that plays. A counted walk has no
            # outro and stores no hand-off.
            current = controller.cur_moving_generator
            if current is not controller.new_moving_generator:
                controller.cur_moving_generator = controller.new_moving_generator
            if controller.cur_moving_generator is None:
                # The walk ended on this call. The published twist
                # has to say the body is stopped.
                controller.linear_x = 0
                controller.linear_y = 0
                controller.angular_z = 0
            return
        # Nothing new is waiting, so the next period reads the
        # whole prepared list. An outro order below overwrites
        # this, and so does the branch that finally starts a
        # generator we stored on an earlier period.
        self.generator_slice = generator_running
        current = controller.cur_moving_generator
        if current is not controller.new_moving_generator:
            if self.generator_outro_order != 'in_effect':
                # The period ended and a different generator is
                # waiting, but the order is not in effect yet, so
                # the generator that is playing stays. The next
                # resumes read the ending section. The waiting
                # generator is copied now, because a later Twist
                # is allowed to change new_moving_generator and
                # is not allowed to change this copy. 'ordered'
                # lasts only until the end of this wake. The tick
                # that sends this wake's feet moves it to
                # 'in_effect', and those feet are still the last
                # target of the slice that just ended. The outro
                # targets are the wakes after this one. The swap
                # branch further down does not run on this call,
                # because the order is 'ordered' and the tick has
                # not moved it yet.
                self.generator_outro_order = 'ordered'
                self.generator_slice = generator_outro
                self.pending_generator_after_outro = (
                    controller.new_moving_generator)
        if controller.cur_moving_generator is None:
            # This method was reached with no generator playing.
            # x and y are cleared. The yaw rate is written as
            # −1 rad/s. No other line treats −1 as a message.
            # A wake that still has a velocity command replaces
            # all three numbers from that command, and a wake
            # with no feet replaces them with zeros. The −1
            # remains only when neither of those writes follows.
            controller.linear_x = 0
            controller.linear_y = 0
            controller.angular_z = -1
        if self.generator_outro_order == 'in_effect':
            # The outro was ordered on an earlier accepted gait
            # step, and that wake already moved the order to
            # 'in_effect'. This is the accepted gait step that
            # starts the generator stored then. The slice returns
            # to the opening section so the new generator begins
            # at its splice, and the order returns to 'off' so a
            # later period is not treated as another ending.
            self.generator_slice = generator_intro
            self.generator_outro_order = 'off'
            controller.cur_moving_generator = (
                self.pending_generator_after_outro)


class _MiniStepInput:
    """What one 20 ms mini-step received from the generator.

    One mini-step is one wake of the loop, in every gait. moving_pose
    is None when this mini-step has no feet. hunter is gait 5.
    same_target_as_last_wake means the gait-5 gate has not
    accepted target_feet_positions, so gen_next_mini_step_yield
    was not called and the cursor did not move.
    ask_for_50_ms_blend is a bool: True asks for a 50 ms servo
    blend on this pose.
    """

    def __init__(self):
        self.moving_pose = None
        self.params = None
        self.ask_for_50_ms_blend = None
        self.last_part = False
        self.same_target_as_last_wake = False
        self.hunter = False


class _GeneratorCall:
    """The three ways StepController resumes a Python generator.

    The class holds no generator. Each method receives the generator
    and calls its send. gen_advance_to_first_yield parks a fresh
    generator on its first yield. gen_next_mini_step_yield takes the
    next walk yield. gen_next_lean_slice takes the next body-lean slice.
    """

    @staticmethod
    def gen_advance_to_first_yield(generator):
        """Run a fresh generator from its first line to its first yield.

        Python requires None on this first resume: no yield is waiting
        to receive a value yet. The None that comes back is discarded.
        Walk generators and the pose transformer both start here.
        No feet exist yet.
        """
        generator.send(None)

    @staticmethod
    def gen_next_mini_step_yield(generator, pose, generator_slice):
        """Resume a walk generator and return its next yield.

        pose is the stored stance. generator_slice is the text from
        the walk state: generator_intro, generator_running, or
        generator_outro. The generator uses it to choose which
        part of its prepared foot list to hand back. The yield is
        six feet, the flag that says this yield closed the period,
        the params, and ask_for_50_ms_blend. The caller handles
        StopIteration when the generator has ended.
        """
        return generator.send((pose, generator_slice))

    @staticmethod
    def gen_next_lean_slice(generator, pose, transform):
        """Resume a pose transformer and return one 20 ms lean slice.

        pose and transform are the current feet and body transform.
        The yield is the shifted feet, the updated transform, and
        whether this slice is the last.
        """
        return generator.send((pose, transform))


class StepController(Node):
    """50 Hz gait/pose loop. Callers queue work on the new_* slots; loop()
    promotes them to cur_* and drives Python generators at 20 ms.

    Each work type uses a pair of slots:
      new_*  — next command, written under self.lock by ROS callbacks
      cur_*  — the generator or tuple the loop thread is driving now
    Pose-set wins: applying a new pose clears every generator (stand, then idle).
    Walking never aborts in the middle of a gait step: the current generator is swapped
    only when it reports last_part (the wrap of one gait step).

    Two walkers share that rule. The legacy walker is MovingGenerator
    and CmdVelGenerator, the walks from before the hunter, and it
    publishes through set_pose_base. The hunter walker is gait 5,
    FollowGaitGenerator, and it can hold one target across mini-steps.
    """
    
    RIPPLE_GAIT = 1
    TRIPOD_GAIT = 2
    
    def __init__(self):
        
        
        super().__init__('step_controller', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self.joints_state = {}

        for value in config.SERVOS.values():
            self.joints_state[value['name']] = 0.0

        self.lock = threading.RLock()

        # Dual-slot command queues. ROS callbacks only write new_*; loop() only
        # reads/clears them. A callback must not mutate a generator while
        # _GeneratorCall is inside its send.
        self.cur_moving_generator = None
        self.new_moving_generator = None
        self.cur_pose_transformer = None
        self.new_pose_transformer = None
        self.cur_actionset_runner = None
        self.new_actionset_runner = None
        self.cur_pose_setter = None
        self.new_pose_setter = None
        self.last_twist = None 

        self.stop = False
        self.pose = build_in_pose.DEFAULT_POSE
        self.org_transform = ((0, 0, 120), (0, 0, 0))
        self.transform = ((0, 0, 120), (0, 0, 0)) #xyz 平移 mm, xyz 欧拉角 rad(Translation in millimeters along the XYZ axes and Euler angles in radians along the XYZ axes)
        self.servo_controller_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.acg = ActionGroupController(self.servo_controller_pub, '/home/ubuntu/software/actionset_editor/ActionGroups')
        self.pose_yaw = 0 # 原始积分偏航角(the raw integrated yaw angle)
        self.real_pose_yaw = None  # 多传感器融合后偏航角(the fused yaw angle from multiple sensors)
        self.position = (0, 0, 0)
        self.angular_z = 0 # z轴上的角速度(the angle velocity along the Z-axis)
        self.voltage = 0.0
        self.linear_x, self.linear_y, self.linear_z = 0, 0, 0 # 三轴上的线性速度(the linear velocity along the three axes (X, Y, and Z))
        self.stopped = threading.Event()  # 正在运行的动作已经被停止标志(a sign indicating that the currently running action has been stopped)
        self.stopping = False  # 停止当前正在运行的任务标志(a sign to stop the currently running task)

        self.cmd_gait = 2
        self.cmd_height = 20
        self.cmd_period = 1.0

        # ROS parameter, not the Python constant. ros2 param set
        # /step_controller use_hexapod_kinematics true changes it.
        # The value is read when a gait-5 generator starts. A change
        # during a gait step does not rebuild the session already walking.
        # Default false keeps kinematics.so. The launch argument of
        # the same name arrives as an override. This node is created
        # with automatically_declare_parameters_from_overrides, so
        # that override is already declared. Declaring it again raises.
        self._declare_if_needed(
            'use_hexapod_kinematics', USE_HEXAPOD_KINEMATICS_DEFAULT)
        self._declare_if_needed('hexapod_ik_trace', TRACE_PATH_DEFAULT)
        self.gait5_session = None
        self._hold_watch = HoldWatch()
        self._second_log = SecondLog()
        use_hexapod = bool(
            self.get_parameter('use_hexapod_kinematics').value)
        trace_path = self.get_parameter('hexapod_ik_trace').value
        limit = read_max_step_rad()
        if limit is None:
            limit = "unavailable"
        self.get_logger().info(
            startup_line(use_hexapod, trace_path, limit))
        # Daemon writer. The 20 ms loop only queues rows.
        self._trace = TraceWriter(trace_path, self.get_logger())

        self.loop_thread = threading.Thread(target=self.loop, daemon=True)
        self.loop_enable = True
        self.loop_thread.start()
        self.joint_control = JointControl()

        self.create_subscription(CmdParam, '/step_controller/cmd_param', self.cmd_param_callback, 1)

    def _declare_if_needed(self, name, default):
        """Declare name unless an override already declared it."""
        if not self.has_parameter(name):
            self.declare_parameter(name, default)

    def destroy_node(self):
        """Stop the 20 ms loop and give the trace a short flush.

        close() waits at most a fifth of a second. A halt is not
        held up by the disk. The loop thread is a daemon; clearing
        loop_enable is what keeps it from queueing another row
        after the writer has gone.
        """
        self.loop_enable = False
        writer = getattr(self, "_trace", None)
        if writer is not None:
            writer.close()
        super().destroy_node()

    def _drop_gait5_if_replaced(self):
        """Forget the session when its generator is no longer current.

        A pose-set, a halt, and an accepted gait step that installed
        another generator all replace cur_moving_generator. The next
        gait-5 start reads the flag again. The hold streak belongs
        to the generator that is walking, so a replacement clears
        that too.
        """
        if self.gait5_session is None:
            return
        if self.gait5_session.generator is self.cur_moving_generator:
            return
        self.gait5_session = None
        self._hold_watch = HoldWatch()

    def _begin_gait5(self, generator, pose):
        """Return this generator's session, creating it on the first mini-step.

        The flag is read here, once. solve_stance_already_stood_on
        solves the stance and does not publish it. The same generator
        keeps this object through shorter points and through a
        gait-step wrap that does not swap the generator.
        """
        if self.gait5_session is not None and self.gait5_session.generator is generator:
            return self.gait5_session
        flag = bool(self.get_parameter('use_hexapod_kinematics').value)
        session, line = start_gait5(generator, flag)
        self.get_logger().info(line)
        session.solve_stance_already_stood_on(pose)
        self.gait5_session = session
        self._hold_watch = HoldWatch()
        return session

    def _record_gait5(self, session, mini_step_result):
        """WARN, the 1 Hz line, and one CSV row. None of these block.

        The row is queued with block=False. A full queue drops it
        inside TraceWriter and warns once a second. The clock is
        the node's clock, so a sim time would land in the same
        column as a wall-clock run.
        """
        t_s = self.get_clock().now().nanoseconds * 1e-9
        self._hold_watch.note(self.get_logger(), mini_step_result)
        self._second_log.note(
            self.get_logger(), t_s, session.backend,
            mini_step_result.what_this_wake_published,
            mini_step_result.joint_change_from_last_sent)
        self._trace.put(trace_row(t_s, session.backend, mini_step_result))

    def _integrate_fraction(self, params, fraction_sent_this_wake):
        """Add twist * 0.02 * fraction_sent_this_wake to the odometry.

        A hold passes a fraction of 0 and adds nothing. The mini-step
        that reaches fraction 1 adds only the leftover fraction. It
        does not repay the seconds the shorter points already
        covered. The velocity fields stored on the node are the
        command scaled by that same fraction, so a shorter point is
        not a full command for one sample. The yaw rotation into
        the world frame stays here. The next target, after
        gen_next_mini_step_yield(), is a fresh 0.02 s of the full twist.
        """
        command_x = params.velocity_x / 1000.0
        command_y = params.velocity_y / 1000.0
        vx, vy, wz, dx, dy, dyaw = odometry_increment(
            command_x, command_y, params.angular_z,
            fraction_sent_this_wake)
        self.linear_x = vx
        self.linear_y = vy
        self.angular_z = wz
        yaw = self.real_pose_yaw if self.real_pose_yaw else self.pose_yaw
        x = math.cos(yaw) * dx - math.sin(yaw) * dy
        y = math.sin(yaw) * dx + math.cos(yaw) * dy
        self.position = (
            self.position[0] + x, self.position[1] + y, self.position[2])
        self.pose_yaw += dyaw

    def _publish_leg_radians(self, angles, duration):
        """Send 18 radians, ids 1..18, in set_leg_position order.

        JointControl applies SERVOS['direction'] and writes the
        pulse. This does not call kinematics.so, and it does not
        replace self.pose. A gait mini-step leaves the stance object
        alone so the generator does not rebake on the next
        gen_next_mini_step_yield().
        """
        joints_data = [
            [joint_id, radian]
            for joint_id, radian in zip(range(1, 19), angles)]
        self.joints_state = self.joint_control.set_multi_joints(
            duration, joints_data, self.joints_state)

    def cmd_param_callback(self, msg):
        """CmdParam: named built-in pose plus the gait/height/period that later cmd_vel will use."""
        with self.lock:
            self.pose = msg.pose  # name string, e.g. 'DEFAULT_POSE' — not a 6-leg tuple
            self.cmd_gait = msg.gait
            self.cmd_height = msg.height
            self.cmd_period = msg.period
            pose =  getattr(build_in_pose, self.pose)
            transform = getattr(build_in_pose, self.pose + '_TRANSFORM')
            self.new_pose_setter = (pose, transform, 2)


    def reset_all_new_gen(self):
        """Drop every queued (not-yet-started) command."""
        self.new_pose_setter = None
        self.new_actionset_runner = None
        self.new_pose_transformer = None
        self.new_moving_generator = None

    def reset_all_cur_gen(self):
        """Drop every in-flight generator. Next loop iteration is idle unless something new is queued."""
        self.cur_pose_setter = None
        self.cur_actionset_runner = None
        self.cur_pose_transformer = None
        self.cur_moving_generator = None

    def loop(self):
        """Run one 20 ms mini-step: a pose command, then exactly one walker.

        The legacy walker is every walk generator from before the
        hunter: MovingGenerator (set_step_mode) and CmdVelGenerator
        (cmd_vel while cmd_gait is not 5). The hunter walker is
        gait 5 only, FollowGaitGenerator. A mini-step with no feet
        clears the published twist and drives neither walker.
        """
        os.system("sudo renice -n -19 -p " + str(os.getpid()))
        # One notebook for the whole thread. The wakes below update
        # it. They do not build a new one, because the outro order
        # has to survive from the wake that writes it to the later
        # wake that starts the stored generator.
        walk_state = _WalkState()
        while self.loop_enable:
            if not self._consume_pose_setter():
                continue
            pose, transform = self._step_pose_transformer()
            mini_step_input = self._pull_mini_step_input(pose, walk_state)
            self._apply_shifted_body_pose(
                pose, transform, mini_step_input.moving_pose)
            if mini_step_input.moving_pose is None:
                self.linear_x, self.linear_y, self.angular_z = 0, 0, 0
            elif mini_step_input.hunter:
                self._tick_hunter_gait(
                    mini_step_input, pose, transform, walk_state)
            else:
                self._tick_legacy_gait(mini_step_input, transform, walk_state)
            self._mark_idle()
            time.sleep(0.02)

    def _consume_pose_setter(self):
        """Apply a queued stand, then drop every generator.

        Returns False when the apply raised. This mini-step stops there,
        including the 20 ms sleep, and the next iteration tries again.
        set_pose(None, None, t) holds the current feet for duration.
        """
        try:
            if self.new_pose_setter is not None:
                self.cur_pose_setter = self.new_pose_setter
                self.new_pose_setter = None
            if self.cur_pose_setter is not None:
                pose, transform, duration = self.cur_pose_setter
                if pose is None or transform is None:
                    self.set_pose_base(self.pose, duration, update_pose=True)
                else:
                    self.set_pose_base(pose, duration, update_pose=True)
                    self.transform = transform
                self.reset_all_cur_gen()
                self.reset_all_new_gen()
        except Exception as e:
            self.get_logger().error("SET POSE " + str(e))
            self.reset_all_cur_gen()
            self.reset_all_new_gen()
            return False
        return True

    def _step_pose_transformer(self):
        """Advance a body lean by one 20 ms slice.

        Returns the pose and transform the walker should read.
        A failed gen_next_lean_slice drops the transformer and keeps
        the stored pose.
        """
        pose = self.pose
        transform = self.transform
        try:
            if self.new_pose_transformer is not None:
                self.cur_pose_transformer = self.new_pose_transformer
                self.new_pose_transformer = None
            if self.cur_pose_transformer is not None:
                pose, transform, last_part = _GeneratorCall.gen_next_lean_slice(
                    self.cur_pose_transformer, pose, transform)
                if last_part:
                    self.cur_pose_transformer = None
        except Exception as e:
            self.get_logger().error("TRANSFORM " + str(e))
            self.cur_pose_transformer = None
        return pose, transform

    def _pull_mini_step_input(self, pose, walk_state):
        """Take one mini-step's input from the current walk generator.

        A queued generator is promoted first, when nothing is
        playing. Legacy generators always call
        gen_next_mini_step_yield, and a yield that closes the
        period updates the walk state before this returns. A
        counted walk swaps on that update. A velocity walk only
        orders its outro. Gait 5 does not take that update here.
        While its gate has not accepted the six feet, the same
        feet are returned and the generator is not resumed,
        because resuming it would drop them. The period-closed
        flag is left for the hunter wake, which can see whether
        the gate actually sent those feet.
        """
        mini_step_input = _MiniStepInput()
        if self.cur_moving_generator is None:
            if self.new_moving_generator is not None:
                self.cur_moving_generator = self.new_moving_generator
        self._drop_gait5_if_replaced()
        if self.cur_moving_generator is None:
            return mini_step_input
        if (self.gait5_session is not None
                and self.gait5_session.target_still_open()):
            six_feet, command, ask_for_50_ms_blend = (
                self.gait5_session.open_target())
            mini_step_input.moving_pose = six_feet
            mini_step_input.params = command
            mini_step_input.ask_for_50_ms_blend = ask_for_50_ms_blend
            mini_step_input.same_target_as_last_wake = True
            mini_step_input.hunter = _is_hunter_gait(mini_step_input.params)
            return mini_step_input
        try:
            yielded = _GeneratorCall.gen_next_mini_step_yield(
                self.cur_moving_generator, pose, walk_state.generator_slice)
            mini_step_input.moving_pose = yielded[0]
            mini_step_input.last_part = yielded[1]
            mini_step_input.params = yielded[2]
            mini_step_input.ask_for_50_ms_blend = yielded[3]
        except StopIteration:
            self.cur_moving_generator = None
            return mini_step_input
        except Exception as e:
            self.get_logger().error("GENERATE ERROR: " + repr(e))
            self.cur_moving_generator = None
            return mini_step_input
        mini_step_input.hunter = _is_hunter_gait(mini_step_input.params)
        if mini_step_input.last_part and not mini_step_input.hunter:
            # The yield just closed a period, and this is not gait 5.
            # The feet already pulled are still the ones this wake
            # sends. A counted walk swaps inside the call. A velocity
            # walk only orders the outro, and the tick below is what
            # moves that order from 'ordered' to 'in_effect' after
            # the servos have been commanded.
            walk_state.after_accepted_gait_step(self, mini_step_input.params)
        return mini_step_input

    def _apply_shifted_body_pose(self, pose, transform, moving_pose):
        """Publish a body-lean slice when the pose object changed.

        The transformer yields a new object. While a walk is also
        running, pseudo=True stores that pose and does not move
        the servos. The walker's mini-step is what moves them.
        """
        if pose is self.pose:
            return
        try:
            self.set_pose_base(
                pose, 0.02, pseudo=(moving_pose is not None),
                update_pose=True)
            self.transform = transform
        except Exception as e:
            self.get_logger().info("POSE " + str(e))
            self.cur_pose_transformer = None

    def _tick_legacy_gait(self, mini_step_input, transform, walk_state):
        """Drive MovingGenerator or CmdVelGenerator for one mini-step.

        This is the walk from before the hunter. Feet go through
        set_pose_base and kinematics.set_leg_position. The opening
        section of a velocity walk blends for 50 ms when the
        generator also asks for that blend. Every other mini-step
        blends for 20 ms. CmdVelParams integrates a full 20 ms of
        the command. A counted walk carries MovingParams, so it
        does not integrate here. A raised error drops the generator.
        """
        try:
            if (walk_state.generator_slice == generator_intro
                    and mini_step_input.ask_for_50_ms_blend):
                # The generator is on its opening section and this
                # pose would jump from the feet of the previous
                # generator. The servos are given 50 ms to arrive.
                # The thread still sleeps 20 ms, so the next wake
                # replaces this pulse.
                self.set_pose_base(
                    mini_step_input.moving_pose, 0.05,
                    pseudo=False, update_pose=False)
            else:
                self.set_pose_base(
                    mini_step_input.moving_pose, 0.02,
                    pseudo=False, update_pose=False)
            if walk_state.generator_outro_order == 'ordered':
                # The pull wrote this order earlier in the same wake,
                # before these feet were sent. The feet belong to the
                # slice that just ended. Moving the order to
                # 'in_effect' means the next resume plays the outro,
                # and the next accepted gait step may start the
                # generator that was stored. The outro feet themselves
                # go out on the wakes after this one.
                walk_state.generator_outro_order = 'in_effect'
            self.transform = transform
            if isinstance(mini_step_input.params, CmdVelParams):
                # mm/s to m/s, then 20 ms of that body velocity into odom.
                self.linear_x = mini_step_input.params.velocity_x / 1000.0
                self.linear_y = mini_step_input.params.velocity_y / 1000.0
                self.angular_z = mini_step_input.params.angular_z
                yaw = self.real_pose_yaw if self.real_pose_yaw else self.pose_yaw
                x = (math.cos(yaw) * self.linear_x * 0.02
                     - math.sin(yaw) * self.linear_y * 0.02)
                y = (math.sin(yaw) * self.linear_x * 0.02
                     + math.cos(yaw) * self.linear_y * 0.02)
                self.position = (
                    self.position[0] + x,
                    self.position[1] + y,
                    self.position[2])
                self.pose_yaw += self.angular_z * 0.02
        except Exception as e:
            self.get_logger().info("MOVING_POSE " + str(e))
            self.cur_moving_generator = None
            self.linear_x, self.linear_y, self.angular_z = 0, 0, 0

    def _tick_hunter_gait(self, mini_step_input, stance, transform, walk_state):
        """Drive gait 5, the hunter follow walk, for one mini-step.

        use_hexapod_kinematics chooses who turns the six tips into
        angles. A legal target whose joint move is too big is
        shortened and the generator waits. Any other refusal
        publishes nothing and does not advance the generator. The
        walk state changes only when the gate has sent the six
        feet that close the period. Odometry adds the fraction of
        the target that this mini-step sent.
        """
        try:
            session = self._begin_gait5(self.cur_moving_generator, stance)
            if not mini_step_input.same_target_as_last_wake:
                session.begin_target(
                    mini_step_input.moving_pose, mini_step_input.params,
                    mini_step_input.ask_for_50_ms_blend,
                    mini_step_input.last_part)
            (mini_step_result, fraction_sent_this_wake,
             gait_step_accepted) = session.mini_step()
            self._record_gait5(session, mini_step_result)
            if gait_step_accepted:
                # The gate sent the six feet, and those feet close
                # the period. A shorter point or a hold leaves this false,
                # so the generator is not asked for its outro while
                # the feet are still on the way to the target. Gait 5
                # is a velocity walk, so this call orders the outro
                # when another generator is waiting. It does not
                # install that generator yet.
                walk_state.after_accepted_gait_step(self, session.params)
            # The opening section blends for 50 ms when the generator
            # asks for it. A later mini-step, including a shorter
            # point of the same target, is 20 ms. The thread still
            # sleeps 20 ms.
            if (walk_state.generator_slice == generator_intro
                    and mini_step_input.ask_for_50_ms_blend):
                duration = 0.05
            else:
                duration = 0.02
            if should_publish(mini_step_result):
                self._publish_leg_radians(mini_step_result.angles, duration)
            if walk_state.generator_outro_order == 'ordered':
                # Same mark as the legacy tick, made after the
                # publish of this wake. The order was written a few
                # lines above. These feet are the wrapping target
                # the gate just accepted, not an outro target.
                # 'in_effect' tells the next resume to play the
                # outro, and tells the next accepted gait step that
                # it may start the stored generator.
                walk_state.generator_outro_order = 'in_effect'
            self.transform = transform
            self._integrate_fraction(
                mini_step_input.params, fraction_sent_this_wake)
        except Exception as e:
            self.get_logger().info("MOVING_POSE " + str(e))
            self.cur_moving_generator = None
            self.linear_x, self.linear_y, self.angular_z = 0, 0, 0

    def _mark_idle(self):
        """Signal stopped when no pose, lean, walk, or action is in flight."""
        idle = (
            self.cur_moving_generator is None
            and self.cur_pose_setter is None
            and self.cur_pose_transformer is None
            and self.cur_actionset_runner is None)
        if idle:
            self.stopped.set()

    def stop_running(self, timeout=0, callback=None):
        """
        停止当前正在执行的任务(stop the currently executing task)
        :param timeout: 超时实际, 超过这个实际还没停止的话直接返回(Timeout limit. If the task is not stopped after this time limit, return directly)
        """
        with self.lock:
            self.reset_all_new_gen() # 将所有现有的新指令清空(clear all existing new commands)
            self.stopping = True
            self.stopped.clear()
            if timeout is None:
                self.stopped.wait()
            elif timeout > 0:
                self.stopped.wait(timeout)
        if callable(callback):
            callback()

    def set_leg_position(self, leg_id, position, duration, pseudo=False, update_pose=False):
        """
        根据输入的指定的腿及末端位置， 计算、设置舵机角度(calculate and set the servo angles based on the specified leg and end effector position)
        此方法将可能更新类成员pose(this method may update the class member 'pose')
        :param leg: 腿的号数(the number of the leg)
        :param position: 末端位置(the end effector position)
        :param duration: 完成此次移动所用时间(the time required to complete this movement)
        :param pseudo: 是否真的执行移动， 若True则只返回计算得到的对应舵机角度而不真正发送控制指令给舵机(Whether to actually execute the movement. If True, only the corresponding servo angles calculated will be returned without actually sending control commands to the servo)
        :param update_pose: 是否更新类成员pose, 此成员记录了机器人的当前姿态(Whether to update the class member 'pose', which records the current posture of the robot)
        :return: 末端位置对应的舵机角度（里(id, 角度）， 中(id, 角度）， 外）, 角度为0-1000的数值(The servo angles corresponding to the end effector position (inside (id, angle), middle (id, angle), outside), with angle values ranging from 0 to 1000)
        """
        self.get_logger().info(f'{leg_id}: {position}')
        joints = kinematics.set_leg_position(leg_id, position)  # 计算新末端位置对应的各个舵机的角度(calculate the angles of each servo corresponding to the new end effector position)
        joints_id_radians = zip([(leg_id - 1) * 3 + i + 1 for i, s in enumerate(joints)], joints)

        if not pseudo:

            new_joints_state = self.joint_control.set_multi_joints(duration, joints_id_radians, self.joints_state)
            self.joints_state = new_joints_state


        if update_pose:
            pose = list(self.pose)
            pose[leg_id - 1] = tuple(position)
            self.pose = tuple(pose)
        return joints

    def set_joint(self, joint_id, radians, duration):
        """
        设置关节角度(set joint angle)
        :param joint_id: 关节id(joint ID)
        :param radians: 关节角度， 单位为弧度(joint angle in the unit of radian)
        :param duration: 完成此动作的用时(the time required to complete this movement)
        """
        new_joints_state = self.joint_control.set_joint(joint_id, radians, duration, self.joints_state)
        self.joints_state = new_joints_state



    def set_leg_relatively(self, leg_id, offset, duration):
        """Move one foot by a xyz offset from the pose currently stored in self.pose."""
        cur_pos = list(self.pose[leg_id - 1])
        new_pos = cur_pos[0] + offset[0], cur_pos[1] + offset[1], cur_pos[2] + offset[2]
        self.set_leg_position(leg_id, new_pos, duration)

    def set_pose_base(self, new_pose, duration, pseudo=False, update_pose=False):
        """
        设置机器人的姿态的基础调用，其他 function 都会调用它(Basic call to set the posture of the robot. It will be called for other functions)
        此方法将更新类成员pose(this method will update the class member 'pose')
        :param new_pose:  机器人的新姿态，六条腿的末端坐标,形如（(x1, y1, z1), (x2, y2, z2),...)(The new posture of the robot and the end effector coordinates of the six legs, in the form of ((x1, y1, z1), (x2, y2, z2), ...))
        :param duration: 完成这次动作所用时间(the time required to complete this movement)
        :param pseudo: 是否真的控制舵机运动， 若为True则只计算并设置相应变量而不真正发送控制指令给舵机(whether to actually control the servo movement. If True, only calculate and set the corresponding variables without actually sending control commands to the servo)
        :return: None
        """
        # 6 legs × 3 joints → 18 servo radians, ids 1..18 in leg order
        joints = [kinematics.set_leg_position(i + 1, position) for i, position in enumerate(new_pose)]
        joints = list(itertools.chain.from_iterable(joints))
        joints_data = [[j, r] for j, r in zip(list(range(1, 19)), joints)]
        


        if not pseudo:
            self.joints_state = self.joint_control.set_multi_joints(duration, joints_data, self.joints_state)
        if update_pose:
            self.pose = tuple(map(tuple, new_pose))

    def set_pose(self, pose, transform, duration, interrupt=True):
        """
        设置机器人的姿态(set the posture of the robot)
        :param pose: 新姿态(new posture)
        :param duration:  完成这次动作所用时间(the time required to complete this movement)
        :return:
        """
        with self.lock:
            if self.pose is None:
                self.new_pose_setter = (None, None, duration)
            else:  
                self.new_pose_setter = (pose, transform, duration)

    
    def set_build_in_pose(self, pose_name, duration, interrupt=True):
        """
        设置机器人的姿态(set the posture of the robot)
        :param pose: 新姿态(new posture)
        :param duration:  完成这次动作所用时间(the time required to complete this movement)
        :return: None
        """

        with self.lock:
            pose =  getattr(build_in_pose, pose_name)
            transform = getattr(build_in_pose, pose_name + '_TRANSFORM')
            self.new_pose_setter = (pose, transform, duration)

    def transform_pose_quaternion(self, translate, quaternion, duration):
        """
        使用平移变换加四元数改变机器人的姿态(change the posture of the robot using translation transformation and quaternion)
        :param translate: 机体中心偏移 (x, y, z)(offset of the center of the robot body (x, y, z))
        :param quaternion: 机体的旋转变换四元数 (x, y, z, w)(quaternion for the rotation and transformation of the robot body)
        :param duration: 完成这个变换的用时(the time required to complete this transformation)
        :return: None
        """
        # 设置新的机器人姿态(set new posture for the robot)
        self.set_pose_base(kinematics_calculate.transform_quat(translate, quaternion), duration) 
    
    def transform_absolutely(self, translate, euler, duration):
        """Queue a body transform whose translation/euler are absolute targets, not deltas."""
        generator = PoseTransformer(PoseTransformerParams(translation=translate, rotation=euler, absolutely=True, duration=duration))
        if generator:
            with self.lock:
                _GeneratorCall.gen_advance_to_first_yield(generator)
                self.new_pose_transformer = generator

    def transform_pose_euler(self, translate, axis, euler, duration, degrees=True):
        """
        使用平移变换加欧拉角改变机器人的姿态(change the posture of the robot using translation transformation and Euler angles)
        :param translate: 机体中心偏移的平移变换 (x, y, z)(the translation transformation for the center offset of the robot (x, y, z))
        :param axis: 欧拉角三个轴的顺序 如 'xyz' 或者 'yzx'(the order of the three axes of Euler angles, such as 'xyz' or 'yzx')
        :param euler: 欧拉角的元组, 顺序要与axis一致(the tuple of Euler angles, in the same order as the axis)
        :param duration: 完成这个变换的用时(the time required to complete this transformation)
        :param degrees: 欧拉角单位是否为角度, True为角度, False为弧度(Whether the unit of Euler angles is degrees. True for degrees, False for radians)
        """
        rotate = R.from_euler(axis, euler, degrees=degrees)  # 用欧拉角建立转换器(use Euler anglr to create transformer)
        r = rotate.as_euler('xyz', degrees=False) # 转换成固定顺序的欧拉角(convert Euler angles to fixed order)
        generator = PoseTransformer(PoseTransformerParams(translation=translate, rotation=r, duration=duration))

        if generator:
            with self.lock:
                _GeneratorCall.gen_advance_to_first_yield(generator)
                self.new_pose_transformer = generator
      
    def cmd_vel(self, twist: Twist):
        # Dead-band on Twist (disabled). If re-enabled, tiny cmd_vel chatter
        # would not rebuild the gait generator.
        # 1. 定义速度变化的阈值(velocity-change thresholds)
        # LINEAR_VEL_THRESHOLD = 0.01  # m/s, 线性速度变化阈值，例如1cm/s
        # ANGULAR_VEL_THRESHOLD = 0.1 # rad/s, 角速度变化阈值，例如约3度/s

        # # 2. 如果上一次的速度存在，则进行判断(if we have a previous Twist, compare against it)
        # if self.last_twist:
        #     # 计算线速度和角速度的变化量(deltas on vx, vy, wz)
        #     linear_x_diff = abs(twist.linear.x - self.last_twist.linear.x)
        #     linear_y_diff = abs(twist.linear.y - self.last_twist.linear.y)
        #     angular_z_diff = abs(twist.angular.z - self.last_twist.angular.z)

        #     # 如果所有变化量都在阈值之内，则认为速度未变，直接返回(all under threshold: ignore this message)
        #     if (linear_x_diff < LINEAR_VEL_THRESHOLD and
        #         linear_y_diff < LINEAR_VEL_THRESHOLD and
        #         angular_z_diff < ANGULAR_VEL_THRESHOLD):
        #         return
        
        # # 3. 如果是第一次接收指令，或者速度变化超过阈值，则更新last_twist(first command or over threshold: store last_twist)
        # # 如果twist是 (0, 0, 0)，表示停止，将last_twist设为None，以便下次任何非零速度都能启动(zero Twist means stop: clear last_twist so the next non-zero always starts a generator)
        # if twist.linear.x == 0 and twist.linear.y == 0 and twist.angular.z == 0:
        #     self.last_twist = None
        # else:
        #     self.last_twist = twist
        
        # Twist is m/s; kinematics / CmdVelParams use mm/s
        linear_x = twist.linear.x*1000  # linear_x 单位为 毫米每秒(linear_x is measured in millimeters per second after this scale; Twist itself is m/s)
        linear_y = twist.linear.y*1000  # linear_y 单位为 毫米每秒(linear_y is measured in millimeters per second after this scale; Twist itself is m/s)
        angular_z = twist.angular.z # 旋转角速度 rad/sec(the speed of rotation angle is rad/sec)


        vel_params = CmdVelParams(
            gait =  self.cmd_gait,
            velocity_x = linear_x,
            velocity_y = linear_y,
            angular_z = angular_z,
            height = self.cmd_height,
            relative_h = False,
            period = self.cmd_period,
            linear_factor = 1.0,
            rotate_factor = 1.0 # 转向时的转向系数， 即给定的值(the turning coefficient during turning, which is the given value)
        )
        # gait 5 is ours. Vendor CmdVelGenerator would call
        # kinematics.cmd_vel_new_point and, worse, set_step_mode if this
        # Twist had been a Traveling walk. Never fall through: default
        # cmd_gait is 2 (vendor tripod).
        if self.cmd_gait == 5:
            generator = FollowGaitGenerator(vel_params, self.get_logger())
        else:
            generator = CmdVelGenerator(vel_params, self.get_logger())
        if generator:
            with self.lock:
                _GeneratorCall.gen_advance_to_first_yield(generator)
                self.new_moving_generator = generator

                self.stop = False


    def set_step_mode(self,
                      gait,
                      amplitude,
                      height,
                      direction,
                      rotation,
                      duration,
                      repeat=1,
                      relative_height=False,
                      rectify=True,
                      integral=True,
                      interrupt=True,
                      feedback_cb=None):

        # gait 11/12/13/15 are not walks: they store cmd_vel defaults.
        # 11 → ripple (1), 12 → tripod (2), 13 → gait 3 if kinematics has one.
        # 15 → our follow gait (5). Do not start a walk on 15.
        if gait == 11 or gait == 12 or gait == 13 or gait == 15:
            self.cmd_period = duration
            self.cmd_gait = gait - 10
            self.cmd_height = height
            if gait == 15:
                self.get_logger().info(
                    'Traveling gait=15: cmd_gait=5 (FollowGaitGenerator). '
                    'Not a walk. Later cmd_vel will not use vendor CmdVelGenerator.'
                )

        else:
            self.set_step_mode_base(gait, amplitude, height, direction, rotation, duration, repeat, relative_height, rectify, integral, feedback_cb)


    def set_step_mode_base(self,
                      gait,
                      amplitude,
                      height,
                      direction,
                      rotation,
                      duration,
                      repeat=1,
                      relative_height=False,
                      rectify=True,
                      integral=True,
                      interrupt = True,
                      feedback_cb=None):
        """
        设置机器人的运动步态(set the robot's walking gait)
        :param gait: 步态(walking gait)
        :param amplitude: 步幅(step length)
        :param height: 步高, 即走路时脚尖的抬起高度(The step height, i.e. the height at which the toe is lifted when walking)
        :param direction: 运动方向(direction of motion)
        :param rotation: 机器人绕机体中心的旋转角速度(the angular velocity of the robot around its center of body during walking)
        :param period: 每步用时(the time taken for each step)
        :param repeat: 要走的步数, 0会一直走下去(the number of steps to take. If set to 0, the robot will keep walking)
        :param relative_height: 步高参数是否为相对高度(whether the step height parameter is a relative height)
        :param rectify: 对实际行走距离的校正参数(the correction parameter for the actual walking distance)
        :param integral: 是否对行走距离进行积分实现里程计(whether to integrate the walking distance to implement odom)
        :param feedback_cb: 运行中状态报告的回调，不建议使用(a callback for reporting the status during operation, which is not recommended to use)
        """

        if gait == 5:
            # Discrete / in-place custom walk. Never MovingGenerator:
            # that path calls kinematics.set_step_mode(..., 5) and the
            # .so does not know 5.
            period = max(float(duration), 1e-3)
            vx = float(amplitude) * math.cos(float(direction)) / period
            vy = float(amplitude) * math.sin(float(direction)) / period
            wz = float(rotation) / period
            generator = FollowGaitGenerator(CmdVelParams(
                gait=5,
                velocity_x=vx,
                velocity_y=vy,
                angular_z=wz,
                height=height,
                relative_h=relative_height,
                period=duration,
            ), self.get_logger())
            self.get_logger().info(
                'FollowGaitGenerator gait=5 (never kinematics.set_step_mode). '
                f'stride={amplitude} mm T={duration}s'
            )
            if generator:
                with self.lock:
                    _GeneratorCall.gen_advance_to_first_yield(generator)
                    self.new_moving_generator = generator
            return

        generator = MovingGenerator(MovingParams(
                gait=gait,
                stride = amplitude,
                height = height,
                direction = direction,
                rotation = rotation,
                period = duration,
                repeat = repeat,
                forever = True if repeat == 0 else False,

                relative_h = relative_height,
                ), self.get_logger())

        if generator:
            with self.lock:
                _GeneratorCall.gen_advance_to_first_yield(generator)
                self.new_moving_generator = generator
