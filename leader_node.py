#!/usr/bin/env python3
"""Leader node for remote LEADER-FOLLOWER teleop — runs where the human driver is.

Reads the reBot Arm 102 leader's joint positions through the lerobot
teleoperator plugin (`rebot_arm_102_leader`) and streams them as an action dict
to the relay, which forwards them to whichever follower node is in the same room.
So you can drive a B601 on the other side of the world; to *see* it, open the
relay's /view URL (the follower node streams the arm's camera there).

Pairs with follower_node.py. Bidirectional: either machine can be leader or
follower.

Run in the conda `lerobot` env (has lerobot + the plugin):
    conda activate lerobot
    pip install websocket-client
    python leader_node.py --relay YOUR_VM:8765 --room b601 --port /dev/ttyUSB0 --id leader1
No-hardware link test (sweeps a gentle action):
    python leader_node.py --relay YOUR_VM:8765 --room b601 --fake
"""
from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parent))          # protocol, relay_link (this dir)

import protocol as P                            # noqa: E402
from relay_link import RelayLink                # noqa: E402

# gentle amplitudes (deg) for --fake, per B601 joint (safe, near zero)
FAKE_JOINTS = {
    "shoulder_pan": 20.0, "shoulder_lift": -15.0, "elbow_flex": -20.0,
    "wrist_flex": 15.0, "wrist_yaw": 20.0, "wrist_roll": 20.0, "gripper": -40.0,
}


def build_leader(port: str, baudrate: int, leader_id: str):
    from lerobot_teleoperator_rebot_arm_102.rebot_arm_102_leader import RebotArm102Leader
    from lerobot_teleoperator_rebot_arm_102.config_rebot_arm_102_leader import (
        RebotArm102LeaderConfig)
    return RebotArm102Leader(RebotArm102LeaderConfig(port=port, baudrate=baudrate, id=leader_id))


def run_leader(args, link: RelayLink, stop) -> None:
    leader = build_leader(args.port, args.baudrate, args.id)
    print(f"[leader] connecting on {args.port} …")
    leader.connect(calibrate=not args.no_calibrate)
    print("[leader] connected. Streaming actions — move the leader arm.")
    view = f"http://{args.relay.split('://')[-1]}/view?room={args.room}"
    print(f"[leader] watch the follower at: {view}")
    period = 1.0 / max(args.fps, 1.0)
    last_log = 0.0
    try:
        while not stop[0]:
            t0 = time.time()
            action = leader.get_action()          # {'<motor>.pos': deg}
            link.send_text(P.action_msg(action, time.time()))
            if t0 - last_log > 1.0:
                last_log = t0
                g = action.get("gripper.pos", 0.0)
                print(f"[leader] streaming ({len(action)} joints), gripper={g:.0f}°   ", end="\r")
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
    finally:
        try:
            leader.disconnect()
        except Exception:   # noqa: BLE001
            pass


def run_fake(args, link: RelayLink, stop) -> None:
    print("[leader] FAKE: sweeping a gentle action (no leader arm).")
    period = 1.0 / max(args.fps, 1.0)
    t0 = time.time()
    while not stop[0]:
        t = time.time() - t0
        action = {f"{m}.pos": amp * np.sin(0.5 * t) for m, amp in FAKE_JOINTS.items()}
        link.send_text(P.action_msg(action, time.time()))
        time.sleep(period)


def main() -> None:
    ap = argparse.ArgumentParser(description="Leader node for remote leader-follower teleop")
    ap.add_argument("--relay", required=True, help="relay host:port (or ws://host:port)")
    ap.add_argument("--room", default=P.DEFAULT_ROOM)
    ap.add_argument("--port", default="/dev/ttyUSB0", help="leader UART port")
    ap.add_argument("--baudrate", type=int, default=1_000_000)
    ap.add_argument("--id", default="leader1", help="lerobot teleop id (calibration file)")
    ap.add_argument("--no-calibrate", action="store_true", help="skip connect-time calibration prompt")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--fake", action="store_true", help="no leader arm; sweep an action (link test)")
    args = ap.parse_args()

    url = P.build_ws_url(args.relay, args.room, "operator")   # leader is the 'operator' role
    link = RelayLink(url, name="leader")
    link.start()
    time.sleep(0.5)

    stop = [False]
    signal.signal(signal.SIGINT, lambda *_a: stop.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_a: stop.__setitem__(0, True))

    try:
        if args.fake:
            run_fake(args, link, stop)
        else:
            run_leader(args, link, stop)
    finally:
        link.stop()


if __name__ == "__main__":
    main()
