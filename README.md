# Ollama ArduCopter SITL harness

A command-line harness for asking an Ollama model about live MAVLink telemetry and changing an ArduCopter simulator's mode, existing waypoints, and Guided destination. Each turn is recorded as JSON Lines with model and tool timings for evaluation.

## Setup

1. Install Python 3.10+ and Ollama. Pull a model that supports tool calling, for example `ollama pull qwen3`.
2. Create a virtual environment and install dependencies:

   ```powershell
   py -m venv .venv
   .\.venv\Scripts\Activate.ps1
   pip install -r requirements.txt
   ```

3. Start Mission Planner's ArduCopter simulator and connect Mission Planner. Configure MAVLink forwarding to send a separate UDP stream to `127.0.0.1:14550` (Mission Planner's MAVLink forwarding/UDP output, or a MAVLink router). Do not bind two applications to the same listening port. The endpoint is configurable; for a TCP server use e.g. `--connect tcp:127.0.0.1:5760`. The connection must carry traffic in both directions. The harness waits for an ArduPilot quadrotor heartbeat; other vehicle types are ignored.
4. Start Ollama and run:

   ```powershell
   python harness.py --model qwen3 --connect udpin:127.0.0.1:14550
   ```

Ask, for example, “What is the current mode and position?”, “Show the mission”, “Change waypoint 2 to latitude -35.363, longitude 149.166, altitude 30 metres”, or “Set GUIDED mode and go to latitude -35.363, longitude 149.166, 30 metres above home.” Type `/quit` to exit. Tool support and reliability vary by model. Ensure model-supplied commands match your intentions: the harness executes tool calls immediately.

## Optional video-assisted arrival

Arrival assistance monitors the latest destination sent by `guided_goto` and offers an explicit handoff to an operator using Mission Planner. Video display and joystick input remain in Mission Planner; no additional Python dependencies or Mission Planner plugins are required by this feature. Without `--approach-radius`, the existing conversational workflow is unchanged.

1. Connect Mission Planner and the harness to the same isolated ArduCopter simulator using separate MAVLink endpoints as described above.
2. Configure and verify video in Mission Planner before the approach. Right-click the HUD and open **Video**: the installed Mission Planner 1.3.83 provides **Set MJPEG source**, **Start Camera**, **Set GStreamer Source**, and **HereLink Video**. The source and any required decoder/runtime must already be available. Follow the [Mission Planner video instructions](https://ardupilot.org/planner/docs/live-video.html) for your source. Keep the video view available; the harness does not open panels, start a camera, or measure video latency/freshness.
3. Configure and verify a compatible controller using **Flight Data → Actions → Joystick**, following the [joystick instructions](https://ardupilot.org/copter/docs/common-joystick.html). Verify its behavior in the simulator before using the handoff. The harness does not discover, calibrate, or enable a controller.
4. Enable assistance with a horizontal approach radius in metres, for example:

   ```powershell
   python harness.py --model qwen3 --connect udpin:127.0.0.1:14550 --approach-radius 30
   ```

The radius is a notification threshold, not a stopping distance or an arrival guarantee. The monitor compares fresh `GLOBAL_POSITION_INT` telemetry against the last Guided destination and notifies once when it is within the radius. Altitude is not part of that comparison. A replacement destination resets the notification. Ordinary mission waypoints are not monitored. Heartbeat and position age must be at most three seconds; missing/stale position data cannot trigger a notification.

### Operator commands

These console commands are handled directly, outside the model, and remain available while an Ollama request is running. Press Enter after each command. Takeover is available before reaching the approach radius.

| Command | Behavior |
| --- | --- |
| `/manual` | Immediately block model movement commands and forget the tracked destination. Request **LOITER** and wait for a new heartbeat confirming the mode, so the previous Guided navigation is no longer active. On failure, remain blocked and report the failure. |
| `/manual-ready` | After LOITER confirmation, acknowledge that **you have checked the Mission Planner controller responds**. The harness records operator confirmation; it cannot independently verify joystick operation. |
| `/resume` | After `/manual-ready`, require a fresh LOITER heartbeat and re-enable commands from new model turns. Leave the vehicle in LOITER and do not replay a destination or resume a mission. Give a new instruction when ready. |
| `/assist` | Show assistance state, the tracked destination, and whether an approach notification has occurred. |
| `/quit` or `/exit` | Close the harness and invalidate pending model commands. This does not change flight mode or stop navigation already active on the simulator. |

LOITER is the supported handoff mode for this initial contribution. It requires suitable positioning; see [ArduPilot's Loiter documentation](https://ardupilot.org/copter/docs/loiter-mode.html). The operator remains responsible for the control configuration and the actual response seen in Mission Planner.

The `/assist` state describes permission for harness commands, not the aircraft's flight mode. In particular, `automation` after `/resume` means new model commands are permitted; it does not mean a mission or destination is active.

While assistance is enabled, one model turn runs at a time in the background. Further questions entered while it is busy are declined, not queued; operator commands still work. A pending HTTP request may take until its timeout to finish, but a takeover invalidates that turn immediately. Its eventual response is discarded and its conversation history reset, preventing an old instruction from being replayed after `/resume`. Read-only questions remain available in operator control once the previous request finishes.

Movement is checked both when dispatching a model tool and immediately before each MAVLink control send, including after geocoding or a mission download. Unexpected mode changes or stale heartbeats also block model movement and require an explicit handoff/resume sequence. This does not establish exclusive authority over other applications: another GCS can still send commands, and an external command that leaves the mode unchanged cannot be inferred from the heartbeat. Use `/manual` explicitly. If takeover interrupts a mission upload, inspect the entire mission in Mission Planner before continuing.

### Video and simulator limits

The basic Mission Planner flight simulator does not configure a rendered onboard camera for this repository. For a scene that responds to simulated movement, supply a camera stream from a visual simulator, such as the [ArduPilot Gazebo camera example](https://ardupilot.org/dev/docs/sitl-with-gazebo.html#streaming-camera-video). A recording or local test stream can verify display plumbing, but cannot validate visual positioning. The harness displays an approach notification directing the operator to the configured Mission Planner view; automatic video-panel activation and video-loss handling are not implemented.

The existing JSONL log additionally contains event records for destinations sent, approaches, handoff requests, mode confirmation, operator confirmation, blocked commands, resumption, and shutdown. Event records have an `event` field; normal turn records retain `prompt`, `tools`, and timing fields. Concurrent records are serialized through one log writer. A turn interrupted during shutdown may have only its shutdown event because the outstanding HTTP worker is a daemon.

## Tests

```powershell
python -m unittest discover -s tests -v
```

The tests require the existing dependencies and use simulated telemetry and mocked model responses; they do not connect to an aircraft, Mission Planner, or Ollama. They cover approach freshness and destination replacement, takeover during delayed model/geocoding/mission operations, mode-confirmation failures, explicit resumption, and behavior with the feature disabled. The console test launches the actual harness against local TCP MAVLink and HTTP test peers and checks operator commands during a pending model response; those peers do not simulate flight dynamics.

For a simulator acceptance check, verify an approach notification for a Guided destination, `/manual` receiving a new LOITER heartbeat, operator input through Mission Planner, and `/resume` leaving the vehicle in LOITER with no destination replay. Also check video independently, interrupt the video source, and confirm that the harness neither claims the feed is healthy nor changes control automatically. A physical controller and a rendered camera stream must be tested separately before claiming the full visual-positioning workflow works.

## Behavior and limits

- Status returns the latest raw MAVLink fields with per-message age; missing or stale data must not be interpreted as current. Message availability/rate depends on the simulator and forwarding configuration. The harness does not request stream rates.
- `get_mission` downloads the mission using MAVLink's mission protocol. Waypoint editing downloads the full mission, changes a single existing `NAV_WAYPOINT` after sequence 0, and uploads the complete mission. Altitude is in the existing item's frame; inspect it before changing altitude. Other mission item fields are retained. Upload completion is reported only after an accepted mission ACK. Do not edit the mission concurrently in Mission Planner. An interrupted upload must be checked in Mission Planner before continuing.
- Mode changes are confirmed by a new heartbeat. Guided goto sends a global relative-altitude position target only when a fresh heartbeat reports GUIDED; it is unacknowledged, so “sent” does not mean reached or accepted. This tool does not arm, take off, or switch modes automatically.
- This is a simulator evaluation tool, not a flight safety system. It has no authentication, geofence, collision avoidance, per-command operator confirmation, or physical-vehicle protection. Connect it only to an isolated simulator. The model can call control tools based on its interpretation of a prompt when enabled by the optional assistance state.
- The log defaults to `runs.jsonl` and contains prompts, tool arguments/results, response text, elapsed wall time, and Ollama's available timing/token counters. It may contain location data. Use `--log PATH` to select a file. Model load and prior conversation context affect timings. Restart the harness for a fresh conversation. Ollama requests time out after 180 seconds and each turn allows up to eight model calls.

Protocol references: [MAVLink mission protocol](https://mavlink.io/en/services/mission.html), [ArduPilot Copter Guided commands](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html), and [Ollama tool calling](https://docs.ollama.com/capabilities/tool-calling).
