# Remote Teleoperation — reBot B601 (leader → follower, over the internet)

Run a **reBot B601 follower arm + at least one camera** anywhere in the world and
drive it from a **reBot Arm 102 leader** somewhere else, watching the live result
in a **browser**. Everything talks through **one self-hosted relay**, so nothing
behind a home router needs port-forwarding — every node and every browser dials
*out* to the relay:

```
 leader_node.py  ──WS: action JSON──►  relay_server.py  ◄──WS: JPEG + state──  follower_node.py
 (reBot Arm 102 leader)                (public cloud VM)                       (B601 RS/DM + ≥1 cam)
                                              │
                                              └── HTTP MJPEG ──►  browser viewers  (open a URL)
```

This is exactly the standard `lerobot-teleoperate` data flow —
`leader.get_action()` → `follower.send_action()` — just split across the
internet. Key properties:

- **Any arm**: RS (RobStride) or DM (Damiao) follower, selected with `--arm rs|dm`.
- **Minimum one camera** on the follower so viewers can see the teleop; a status
  HUD (joint angles, FOLLOWING/HOLD) is drawn onto the video, so the browser page
  is just an `<img>`.
- **Bidirectional**: either machine can be the leader or the follower — drive
  their arm or let them drive yours.
- **NAT-friendly**: only the relay needs a public IP.

## Components

| File | Runs where | What it does |
|------|-----------|--------------|
| `relay_server.py` | a cloud VM with a public IP | rooms; forwards leader→follower actions + follower→viewer state; re-serves the follower camera as browser MJPEG |
| `follower_node.py` | next to the physical arm | connects the B601 RS/DM follower, applies actions with the lerobot plugin, streams a camera |
| `leader_node.py` | where the driver is | reads the reBot Arm 102 leader and streams its joint positions |
| `viewer.html` | served by the relay | the browser page (`/view?room=…`) |
| `protocol.py`, `relay_link.py`, `camera.py` | both nodes | wire protocol, auto-reconnecting WebSocket client, camera source |

## Requirements

**Relay** (any Python 3.9+ box with a public IP):
```bash
pip install -r requirements-relay.txt        # aiohttp
```

**Leader / follower nodes** run in a Python env that already has **LeRobot + the
reBot plugins + motorbridge** (the standard `lerobot-teleoperate` setup):

- [LeRobot](https://github.com/huggingface/lerobot)
- [lerobot-robot-seeed-b601](https://github.com/Seeed-Projects/lerobot-robot-seeed-b601) — the `seeed_b601_rs_follower` / `seeed_b601_dm_follower` robot
- [lerobot-teleoperator-rebot-arm-102](https://github.com/tianrking/lerobot-teleoperator-rebot-arm-102) — the `rebot_arm_102_leader` teleoperator
- `motorbridge` (matching wheel for your platform/Python)

Then add this project's only extra dependency:
```bash
pip install -r requirements-node.txt          # websocket-client (opencv/numpy already come with lerobot)
```

## 1. Start the relay (once, on a public VM)

```bash
python relay_server.py --port 8765            # binds 0.0.0.0:8765
```
Open the port in the VM firewall. Everyone uses `YOUR_VM_IP:8765`. For HTTPS/WSS,
put nginx/Caddy TLS in front and use `wss://`/`https://`.

## 2. Run the follower (where the arm is)

```bash
# RobStride follower on SocketCAN:
sudo ip link set can0 up type can bitrate 1000000
python follower_node.py --relay YOUR_VM_IP:8765 --room b601 \
    --arm rs --port can0 --can-adapter socketcan --id follower1 --camera 0

# Damiao follower over its serial bridge instead:
python follower_node.py --relay YOUR_VM_IP:8765 --room b601 \
    --arm dm --port /dev/ttyACM0 --can-adapter damiao --id follower1 --camera 0
```
`--camera` is any cv2 index (or `test` for a synthetic pattern). First connect
may prompt for calibration (same as `lerobot-teleoperate`); pass `--no-calibrate`
to reuse an existing calibration file.

## 3. Run the leader (where the driver is)

```bash
python leader_node.py --relay YOUR_VM_IP:8765 --room b601 --port /dev/ttyUSB0 --id leader1
```
Move the leader arm → the follower mirrors it.

## 4. Watch from anywhere

Open **`http://YOUR_VM_IP:8765/view?room=b601`** in any browser (phone/laptop).
The landing page `http://YOUR_VM_IP:8765/` lists active rooms.

## Try it with no hardware

Validates relay + video + control end-to-end with a synthetic camera and a swept
fake action — no arm, no leader, no webcam:

```bash
python relay_server.py  --port 8765 &
python follower_node.py --relay 127.0.0.1:8765 --room demo --fake --camera test
python leader_node.py   --relay 127.0.0.1:8765 --room demo --fake
# watch: http://127.0.0.1:8765/view?room=demo   → HUD shows the swept joint values + FOLLOWING
```

## Safety (follower)

- every action is **clipped to the follower's configured `joint_limits`** (the
  lerobot plugin's `send_action`)
- **`--max-relative-target DEG`** caps how far each joint may move per update
  (default 8°) so a large leader/follower pose gap — or a bad packet — slews
  gently instead of lurching
- **watchdog**: if no action arrives within `--watchdog` s (link drop / pause),
  the follower stops sending and POS_VEL holds the last commanded pose
- `Ctrl+C` disconnects (the plugin disables torque on disconnect by default)

## Notes

- Video is MJPEG relayed through the server (no WebRTC/STUN/TURN), so it goes
  wherever plain HTTP goes. For many viewers or cellular links, lower `--fps`,
  `--jpeg-quality`, and `--width/--height` on the follower.
- `room` is any string — different leader/follower pairs can share one relay by
  using different room names.
- One arm per room; multiple browser viewers; the latest action wins.
