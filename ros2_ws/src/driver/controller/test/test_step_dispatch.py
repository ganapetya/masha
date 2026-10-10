# The 20 ms loop picks one walker. These tests lock that pick
# without a node and without the thread: legacy generators still
# resume through gen_next_mini_step_yield and publish through
# set_pose_base. Gait 5 does not resume while a target is waiting.
# The fake generator still implements send(), because that is what
# _GeneratorCall calls.

import pytest

# Imported from the test body, not at collection. test_leg_ik checks
# that `kinematics` is still unloaded when its first test starts, and
# step_controller imports that module.
StepController = None
_MiniStepInput = None
_WalkState = None
_is_hunter_gait = None
CmdVelParams = None
MovingParams = None
generator_intro = None
generator_running = None
generator_outro = None


@pytest.fixture(autouse=True)
def _load_controller():
    global StepController, _MiniStepInput, _WalkState, _is_hunter_gait
    global CmdVelParams, MovingParams
    global generator_intro, generator_running, generator_outro
    from controller.move import CmdVelParams as cmd_params
    from controller.move import MovingParams as move_params
    from controller.move import generator_intro as intro_text
    from controller.move import generator_running as running_text
    from controller.move import generator_outro as outro_text
    from controller.step_controller import StepController as controller
    from controller.step_controller import _MiniStepInput as input_type
    from controller.step_controller import _WalkState as walk_state_type
    from controller.step_controller import _is_hunter_gait as hunter_check
    StepController = controller
    _MiniStepInput = input_type
    _WalkState = walk_state_type
    _is_hunter_gait = hunter_check
    CmdVelParams = cmd_params
    MovingParams = move_params
    generator_intro = intro_text
    generator_running = running_text
    generator_outro = outro_text


class _Log:
    def error(self, text):
        self.text = text

    def info(self, text):
        self.text = text


class _Hold:
    """A gait-5 session whose open target is not finished."""

    def __init__(self, target_feet_positions, params, slow):
        self.target_feet_positions = target_feet_positions
        self.params = params
        self.slow = slow
        self.sends = 0

    def waiting(self):
        return True

    def held(self):
        return self.target_feet_positions, self.params, self.slow


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
    """Build the fields the pull and the two mini-steps read. Not a ROS node."""
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
    _WalkState().after_accepted_gait_step(step, MovingParams(gait=2))
    assert step.cur_moving_generator is nxt
    assert step.linear_x == 1


def test_discrete_boundary_with_nothing_queued_zeros_twist():
    step = _step()
    step.cur_moving_generator = object()
    step.new_moving_generator = None
    _WalkState().after_accepted_gait_step(step, MovingParams(gait=2))
    assert step.cur_moving_generator is None
    assert (step.linear_x, step.linear_y, step.angular_z) == (0, 0, 0)


def test_cmd_vel_boundary_winds_down_before_the_swap():
    step = _step()
    current = object()
    nxt = object()
    step.cur_moving_generator = current
    step.new_moving_generator = nxt
    walk_state = _WalkState()
    walk_state.after_accepted_gait_step(step, CmdVelParams(gait=2))
    assert step.cur_moving_generator is current
    assert walk_state.generator_slice == generator_outro
    assert walk_state.generator_outro_order == 'ordered'
    assert walk_state.pending_generator_after_outro is nxt
    # The tick would make this move after sending the feet. The
    # test does that write itself, then asks for the next accepted
    # gait step, which is the one that starts the stored generator.
    walk_state.generator_outro_order = 'in_effect'
    walk_state.after_accepted_gait_step(step, CmdVelParams(gait=2))
    assert step.cur_moving_generator is nxt
    assert walk_state.generator_slice == generator_intro
    assert walk_state.generator_outro_order == 'off'


def test_legacy_last_part_is_honored_before_the_tick():
    step = _step()
    nxt = object()
    feet = ((1, 2, 3),)
    step.cur_moving_generator = _Gen((feet, True, MovingParams(gait=2), False))
    step.new_moving_generator = nxt
    mini_step_input = StepController._pull_mini_step_input(step, 'pose', _WalkState())
    assert mini_step_input.hunter is False
    assert mini_step_input.moving_pose == feet
    assert step.cur_moving_generator is nxt


def test_hunter_last_part_does_not_swap_during_the_pull():
    step = _step()
    gen = _Gen(((1, 2, 3), True, CmdVelParams(gait=5), False))
    nxt = object()
    step.cur_moving_generator = gen
    step.new_moving_generator = nxt
    mini_step_input = StepController._pull_mini_step_input(step, 'pose', _WalkState())
    assert mini_step_input.hunter is True
    assert mini_step_input.last_part is True
    assert step.cur_moving_generator is gen
    assert gen.sent == [('pose', generator_intro)]


def test_waiting_hunter_target_is_not_sent_again():
    step = _step()
    params = CmdVelParams(gait=5)
    feet = ((4, 5, 6),)
    step._gait5 = _Hold(feet, params, False)
    gen = _Gen(error=AssertionError(
        'gen_next_mini_step_yield dropped the open target'))
    step.cur_moving_generator = gen
    mini_step_input = StepController._pull_mini_step_input(step, 'pose', _WalkState())
    assert mini_step_input.hunter is True
    assert mini_step_input.reused is True
    assert mini_step_input.moving_pose == feet
    assert gen.sent == []


def test_vendor_cmd_vel_is_a_legacy_mini_step():
    step = _step()
    params = CmdVelParams(gait=2, velocity_x=100)
    feet = ((1, 0, 0),)
    gen = _Gen((feet, False, params, False))
    step.cur_moving_generator = gen
    mini_step_input = StepController._pull_mini_step_input(step, 'pose', _WalkState())
    assert mini_step_input.hunter is False
    assert mini_step_input.moving_pose == feet


def test_a_dead_generator_leaves_no_feet():
    step = _step()
    step.cur_moving_generator = _Gen(error=StopIteration())
    mini_step_input = StepController._pull_mini_step_input(step, 'pose', _WalkState())
    assert mini_step_input.moving_pose is None
    assert step.cur_moving_generator is None


def test_legacy_tick_uses_set_pose_base_and_a_full_20_ms():
    step = _step()
    calls = []
    step.set_pose_base = lambda *args, **kwargs: calls.append((args, kwargs))
    step.real_pose_yaw = 0.0
    step.pose_yaw = 0.0
    step.position = (0.0, 0.0, 0.0)
    step.cur_moving_generator = object()
    mini_step_input = _MiniStepInput()
    mini_step_input.moving_pose = ((1, 2, 3),)
    mini_step_input.params = CmdVelParams(gait=2, velocity_x=1000, velocity_y=0, angular_z=0)
    mini_step_input.slow = False
    walk_state = _WalkState()
    walk_state.generator_slice = generator_running
    StepController._tick_legacy_gait(step, mini_step_input, 'lean', walk_state)
    assert calls == [(
        (mini_step_input.moving_pose, 0.02),
        {'pseudo': False, 'update_pose': False})]
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
    mini_step_input = _MiniStepInput()
    mini_step_input.moving_pose = ((1, 2, 3),)
    mini_step_input.params = MovingParams(gait=2)
    mini_step_input.slow = False
    StepController._tick_legacy_gait(step, mini_step_input, 'lean', _WalkState())
    assert calls == [0.02]
    assert step.linear_x == 1

    mini_step_input.slow = True
    mini_step_input.params = CmdVelParams(gait=2, velocity_x=0, velocity_y=0, angular_z=0)
    StepController._tick_legacy_gait(step, mini_step_input, 'lean', _WalkState())
    assert calls == [0.02, 0.05]


def test_hunter_mini_step_begins_a_new_target_and_publishes_radians():
    step = _step()
    opened = []
    published = []
    integrated = []

    class Session:
        slow = False

        def begin_target(self, pose, params, slow, last_part):
            opened.append((pose, last_part))

        def mini_step(self):
            return _StepResult(True), 0.5, False

    step._begin_gait5 = lambda generator, stance: Session()
    step._record_gait5 = lambda session, result: None
    step._publish_leg_radians = lambda angles, duration: published.append(duration)
    step._integrate_fraction = lambda params, delta: integrated.append(delta)
    step.cur_moving_generator = object()
    mini_step_input = _MiniStepInput()
    mini_step_input.moving_pose = ((9, 9, 9),)
    mini_step_input.params = CmdVelParams(gait=5)
    mini_step_input.slow = False
    mini_step_input.last_part = True
    mini_step_input.hunter = True
    StepController._tick_hunter_gait(step, mini_step_input, 'stance', 'lean', _WalkState())
    assert opened == [(((9, 9, 9),), True)]
    assert published == [0.02]
    assert integrated == [0.5]
    assert step.transform == 'lean'


def test_hunter_reuse_does_not_open_and_a_boundary_winds_down():
    step = _step()
    opened = []
    current = object()
    nxt = object()
    step.cur_moving_generator = current
    step.new_moving_generator = nxt

    class Session:
        # Gait 5 is a velocity walk. An accepted gait step orders
        # the outro. It does not install the queued generator on
        # this same call.
        params = CmdVelParams(gait=5)
        slow = False

        def begin_target(self, *args):
            opened.append(args)

        def mini_step(self):
            return _StepResult(False), 0.0, True

    step._begin_gait5 = lambda generator, stance: Session()
    step._record_gait5 = lambda session, result: None
    step._publish_leg_radians = lambda angles, duration: None
    step._integrate_fraction = lambda params, delta: None
    mini_step_input = _MiniStepInput()
    mini_step_input.moving_pose = ((1, 1, 1),)
    mini_step_input.params = CmdVelParams(gait=5)
    mini_step_input.slow = False
    mini_step_input.reused = True
    walk_state = _WalkState()
    StepController._tick_hunter_gait(step, mini_step_input, 'stance', 'lean', walk_state)
    assert opened == []
    assert step.cur_moving_generator is current
    assert walk_state.pending_generator_after_outro is nxt
    assert walk_state.generator_slice == generator_outro
    # The tick both orders the outro and, after the publish,
    # moves the order to in_effect. 'ordered' does not outlive
    # the wake.
    assert walk_state.generator_outro_order == 'in_effect'


class _StepResult:
    def __init__(self, publish):
        self.publish = publish
        self.angles = (0.1,) * 18
        self.decision = 'sent'
        self.dq = ()
