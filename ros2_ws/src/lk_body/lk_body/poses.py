"""Jaw while a line plays. No rclpy. Stance poses are a later step."""

import logging
import time

log = logging.getLogger("lk_body.poses")

LOCK_S = 4.0
START_GRACE_S = 0.3


def jaw_while(still_playing, publish, complete, name: str) -> None:
    """Publish the speaking action again while the line is still playing.

    complete() is true when the controller is idle. A missing start at 300 ms
    logs not-started and does not send stop. When the line ends, this does not
    publish again and does not send stop.
    """
    logged = False
    while still_playing():
        publish(name)
        started = _wait_until(lambda: not complete(), START_GRACE_S, still_playing)
        if not started:
            if not logged:
                log.info("not-started %s", name)
                logged = True
            _wait_until(lambda: False, 0.5, still_playing)
            continue
        deadline = time.monotonic() + LOCK_S
        while still_playing() and time.monotonic() < deadline:
            if complete():
                break
            time.sleep(0.05)
    # The line has ended. Do not publish again and do not send stop.


def _wait_until(predicate, seconds: float, still_playing) -> bool:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and still_playing():
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()
