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

## Behavior and limits

- Status returns the latest raw MAVLink fields with per-message age; missing or stale data must not be interpreted as current. Message availability/rate depends on the simulator and forwarding configuration. The harness does not request stream rates.
- `get_mission` downloads the mission using MAVLink's mission protocol. Waypoint editing downloads the full mission, changes a single existing `NAV_WAYPOINT` after sequence 0, and uploads the complete mission. Altitude is in the existing item's frame; inspect it before changing altitude. Other mission item fields are retained. Upload completion is reported only after an accepted mission ACK. Do not edit the mission concurrently in Mission Planner. An interrupted upload must be checked in Mission Planner before continuing.
- Mode changes are confirmed by a new heartbeat. Guided goto sends a global relative-altitude position target only when a fresh heartbeat reports GUIDED; it is unacknowledged, so “sent” does not mean reached or accepted. This tool does not arm, take off, or switch modes automatically.
- This is a simulator evaluation tool, not a flight safety system. It has no authentication, geofence, collision avoidance, operator confirmation, or physical-vehicle protection. Connect it only to an isolated simulator. The model can call control tools based on its interpretation of a prompt.
- The log defaults to `runs.jsonl` and contains prompts, tool arguments/results, response text, elapsed wall time, and Ollama's available timing/token counters. It may contain location data. Use `--log PATH` to select a file. Model load and prior conversation context affect timings. Restart the harness for a fresh conversation. Ollama requests time out after 180 seconds and each turn allows up to eight model calls.

Protocol references: [MAVLink mission protocol](https://mavlink.io/en/services/mission.html), [ArduPilot Copter Guided commands](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html), and [Ollama tool calling](https://docs.ollama.com/capabilities/tool-calling).
