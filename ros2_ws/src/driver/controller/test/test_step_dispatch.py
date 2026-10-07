# The 20 ms loop picks one walker. These tests lock that pick
# without a node and without the thread: legacy generators still
# send and publish through set_pose_base, gait 5 does not send
# while a frame is waiting.

import pytest

# Imported from the test body, not at collection. test_leg_ik checks
# that `kinematics` is still unloaded when its first test starts, and
# step_controller imports that module.
StepController = None
_MovingFrame = None
_WalkHandshake = None
_is_hunter_gait = None
CmdVelParams = None
MovingParams = None


@pytest.fixture(autouse=True)
def _load_controller():
    global StepController, _MovingFrame, _WalkHandshake, _is_hunter_gait
    global CmdVelParams, MovingParams
    from controller.move import CmdVelParams as cmd_params
    from controller.move import MovingParams as move_params
    from controller.step_controller import StepController as controller
    from controller.step_controller import _MovingFrame as frame_type
    from controller.step_controller import _WalkHandshake as handshake_type
    from controller.step_controller import _is_hunter_gait as hunter_check
    StepController = controller
    _MovingFrame = frame_type
    _WalkHandshake = handshake_type
    _is_hunter_gait = hunter_check
    CmdVelParams = cmd_params
    MovingParams = move_params


class _Log:
    def error(self, text):
        self.text = text

    def info(self, text):
        self.text = text


class _Hold:
    """A gait-5 session whose open frame is not finished."""

    def __init__(self, frame, params, slow):
        self.frame = frame
        self.params = params
        self.slow = slow
        self.sends = 0

    def waiting(self):
        return True

    def held(self):
        return self.frame, self.params, self.slow


class _Gen:
    def __init__(self, result=None, error=None):
        self.result = result
        self.error = error
        self.sent = []

    def send(self, value):
        self.sent.append(value)
        if self.error is not None:
            raise self.error
        return self.result


def _step():
    """Build the fields pull and the two ticks read. Not a ROS node."""
    robot = _Log()
    robot.cur_moving_generator = None
    robot.new_moving_generator = None
    robot._gait5 = None
    robot.linear_x = 1
    robot.linear_y = 2
    robot.angular_z = 3
    robot.get_logger = lambda: robot
    robot._drop_gait5_if_replaced = lambda: None
    return robot


def test_only_gait_5_is_the_hunter():
    assert _is_hunter_gait(CmdVelParams(gait=5))
    assert not _is_hunter_gait(CmdVelParams(gait=2))
    assert not _is_hunter_gait(MovingParams(gait=1))
    assert not _is_hunter_gait(MovingParams(gait=2))
    assert not _is_hunter_gait(None)


def test_discrete_boundary_swaps_immediately():
    step = _step()
    nxt = object()
    step.cur_moving_generator = object()
    step.new_moving_generator = nxt
    _WalkHandshake().honor_boundary(step, 'move')
    assert step.cur_moving_generator is nxt
    assert step.linear_x == 1


def test_discrete_boundary_with_nothing_queued_zeros_twist():
    step = _step()
    step.cur_moving_generator = object()
    step.new_moving_generator = None
    _WalkHandshake().honor_boundary(step, 'move')
    assert step.cur_moving_generator is None
    assert (step.linear_x, step.linear_y, step.angular_z) == (0, 0, 0)


def test_cmd_vel_boundary_winds_down_before_the_swap():
    step = _step()
    current = object()
    nxt = object()
    step.cur_moving_generator = current
    step.new_moving_generator = nxt
    handshake = _WalkHandshake()
    handshake.honor_boundary(step, 'cmd_false')
    assert step.cur_moving_generator is current
    assert handshake.send_status == 'finish'
    assert handshake.last_status == 'stop'
    assert handshake.pending is nxt
    handshake.last_status = 'finish'
    handshake.honor_boundary(step, 'cmd_false')
    assert step.cur_moving_generator is nxt
    assert handshake.send_status == 'first'
    assert handshake.last_status == 'start'


def test_legacy_last_part_is_honored_before_the_tick():
    step = _step()
    nxt = object()
    feet = ((1, 2, 3),)
    step.cur_moving_generator = _Gen((feet, True, MovingParams(gait=2), 'move'))
    step.new_moving_generator = nxt
    frame = StepController._pull_moving_frame(step, 'pose', _WalkHandshake())
    assert frame.hunter is False
    assert frame.moving_pose == feet
    assert step.cur_moving_generator is nxt


def test_hunter_last_part_does_not_swap_during_the_pull():
    step = _step()
    gen = _Gen(((1, 2, 3), True, CmdVelParams(gait=5), 'cmd_false'))
    nxt = object()
    step.cur_moving_generator = gen
    step.new_moving_generator = nxt
    frame = StepController._pull_moving_frame(step, 'pose', _WalkHandshake())
    assert frame.hunter is True
    assert frame.last_part is True
    assert step.cur_moving_generator is gen
    assert gen.sent == [('pose', 'first')]


def test_waiting_hunter_frame_is_not_sent_again():
    step = _step()
    params = CmdVelParams(gait=5)
    feet = ((4, 5, 6),)
    step._gait5 = _Hold(feet, params, 'cmd_false')
    gen = _Gen(error=AssertionError('send dropped the open frame'))
    step.cur_moving_generator = gen
    frame = StepController._pull_moving_frame(step, 'pose', _WalkHandshake())
    assert frame.hunter is True
    assert frame.reused is True
    assert frame.moving_pose == feet
    assert gen.sent == []


def test_vendor_cmd_vel_is_a_legacy_frame():
    step = _step()
    params = CmdVelParams(gait=2, velocity_x=100)
    feet = ((1, 0, 0),)
    gen = _Gen((feet, False, params, 'cmd_false'))
    step.cur_moving_generator = gen
    frame = StepController._pull_moving_frame(step, 'pose', _WalkHandshake())
    assert frame.hunter is False
    assert frame.moving_pose == feet


def test_a_dead_generator_leaves_no_feet():
    step = _step()
    step.cur_moving_generator = _Gen(error=StopIteration())
    frame = StepController._pull_moving_frame(step, 'pose', _WalkHandshake())
    assert frame.moving_pose is None
    assert step.cur_moving_generator is None


def test_legacy_tick_uses_set_pose_base_and_a_full_20_ms():
    step = _step()
    calls = []
    step.set_pose_base = lambda *args, **kwargs: calls.append((args, kwargs))
    step.real_pose_yaw = 0.0
    step.pose_yaw = 0.0
    step.position = (0.0, 0.0, 0.0)
    step.cur_moving_generator = object()
    frame = _MovingFrame()
    frame.moving_pose = ((1, 2, 3),)
    frame.params = CmdVelParams(gait=2, velocity_x=1000, velocity_y=0, angular_z=0)
    frame.slow = 'cmd_false'
    handshake = _WalkHandshake()
    handshake.send_status = 'running'
    StepController._tick_legacy_gait(step, frame, 'lean', handshake)
    assert calls == [((frame.moving_pose, 0.02), {'pseudo': False, 'update_pose': False})]
    assert step.linear_x == pytest.approx(1.0)
    assert step.position[0] == pytest.approx(0.02)
    assert step.transform == 'lean'
    assert step.cur_moving_generator is not None


def test_legacy_first_slice_blends_for_50_ms_and_a_move_does_not_integrate():
    step = _step()
    calls = []
    step.set_pose_base = lambda *args, **kwargs: calls.append(args[1])
    step.real_pose_yaw = None
    step.pose_yaw = 0.0
    step.position = (0.0, 0.0, 0.0)
    frame = _MovingFrame()
    frame.moving_pose = ((1, 2, 3),)
    frame.params = MovingParams(gait=2)
    frame.slow = 'move'
    StepController._tick_legacy_gait(step, frame, 'lean', _WalkHandshake())
    assert calls == [0.02]
    assert step.linear_x == 1

    frame.slow = 'cmd_true'
    frame.params = CmdVelParams(gait=2, velocity_x=0, velocity_y=0, angular_z=0)
    StepController._tick_legacy_gait(step, frame, 'lean', _WalkHandshake())
    assert calls == [0.02, 0.05]


def test_hunter_tick_opens_a_new_frame_and_publishes_radians():
    step = _step()
    opened = []
    published = []
    integrated = []

    class Session:
        slow = 'cmd_false'

        def open_frame(self, pose, params, slow, last_part):
            opened.append((pose, last_part))

        def step(self):
            return _StepResult(True), 0.5, False

    step._begin_gait5 = lambda generator, stance: Session()
    step._record_gait5 = lambda session, result: None
    step._publish_leg_radians = lambda angles, duration: published.append(duration)
    step._integrate_fraction = lambda params, delta: integrated.append(delta)
    step.cur_moving_generator = object()
    frame = _MovingFrame()
    frame.moving_pose = ((9, 9, 9),)
    frame.params = CmdVelParams(gait=5)
    frame.slow = 'cmd_false'
    frame.last_part = True
    frame.hunter = True
    StepController._tick_hunter_gait(step, frame, 'stance', 'lean', _WalkHandshake())
    assert opened == [(((9, 9, 9),), True)]
    assert published == [0.02]
    assert integrated == [0.5]
    assert step.transform == 'lean'


def test_hunter_reuse_does_not_open_and_a_boundary_swaps():
    step = _step()
    opened = []
    current = object()
    nxt = object()
    step.cur_moving_generator = current
    step.new_moving_generator = nxt

    class Session:
        slow = 'move'

        def open_frame(self, *args):
            opened.append(args)

        def step(self):
            return _StepResult(False), 0.0, True

    step._begin_gait5 = lambda generator, stance: Session()
    step._record_gait5 = lambda session, result: None
    step._publish_leg_radians = lambda angles, duration: None
    step._integrate_fraction = lambda params, delta: None
    frame = _MovingFrame()
    frame.moving_pose = ((1, 1, 1),)
    frame.params = CmdVelParams(gait=5)
    frame.slow = 'cmd_false'
    frame.reused = True
    StepController._tick_hunter_gait(step, frame, 'stance', 'lean', _WalkHandshake())
    assert opened == []
    assert step.cur_moving_generator is nxt


class _StepResult:
    def __init__(self, publish):
        self.publish = publish
        self.angles = (0.1,) * 18
        self.decision = 'sent'
        self.dq = ()
