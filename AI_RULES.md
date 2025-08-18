# Hot Potato Game – ESP-IDF 5.3.3 Rules

## Environment
- Target: ESP32 (M5Stack FIRE)
- Framework: ESP-IDF v5.3.3 ONLY
- Tooling: Build/Flash via idf.py (no Arduino APIs, no delay())
- Language: C/C++ (use ESP-IDF APIs and FreeRTOS where applicable)

## Game Concept
- Multiplayer "Hot Potato" using:
  - IR (2400 bps UART-style OOK, 38 kHz modulation) for passes
  - ESP-NOW for pass acknowledgment
  - Planned BLE RSSI for proximity detection
- No tagbacks: a player can’t pass back immediately
- Health decreases while holding potato, last player alive wins

## IR Protocol
- Bitrate: 2400 bps (bit = ~416µs)
- OOK Encoding:
  - Logic 0 = 38 kHz LEDC modulated pulse
  - Logic 1 = gap (no carrier)
- Preamble: ASCII "ZT" (0x5A, 0x54)
- Packet layout (after preamble):
  1. Type (1 byte)
  2. Sender MAC (6 bytes)
  3. Sequence number (1 byte)
  4. CRC8 (1 byte)
- MSB-first bit order, UART-style framing if needed
- Self-filter: ignore packets from own MAC
- Optional: replay protection using sequence number

## Hardware Requirements
- No deprecated APIs, must compile cleanly on ESP-IDF v5.3.3

## Architecture
- Separate game logic from hardware drivers
- Use FreeRTOS tasks and queues for:
  - IR RX task
  - IR TX task
  - ESP-NOW task
  - Game state task
- Logging: Use ESP_LOGx with tags ("IR_TX", "IR_RX", "NOW", "GAME")

## Coding Rules
- No delay() / delayMicroseconds(); use hardware timers (GPTimer) or RMT
- Avoid using features from ESP-IDF > 5.3.3
- Keep functions modular and testable
- Provide debug logging and clear inline comments

## Deliverables
- Functions:
  - `ir_send_pass(sender_mac, seq)`
  - `ack_send_esnow(dest_mac, seq)`
  - IR RX callback parsing packet, validating CRC8, and sending event to game logic
- Integration: ACK wait timer in game logic to confirm passes

## Task Priorities (FreeRTOS)
- **IR RX Task** → **High Priority** (e.g., `configMAX_PRIORITIES - 1`)
  - Must not miss incoming bits.
  - Runs in real-time, minimal processing; pushes parsed packets into a queue.

- **IR TX Task** → **High-Mid Priority**
  - Needs accurate timing to send bits.
  - Can be preempted by IR RX but not by slow logic.

- **ESP-NOW Task** → **Mid Priority**
  - Wireless ACKs are important but not as timing-critical as IR.
  - Should be responsive within a few milliseconds.

- **Game State Task** → **Normal Priority**
  - Processes events from queues.
  - Updates LEDs, sounds, and health timers.
  - Can yield to allow hardware tasks to run first.

- **Notes**
  - Keep hardware tasks short — move heavy processing to game logic.
  - Use queues to pass events (e.g., `PASS_EVENT`, `ACK_EVENT`) between tasks.
  - Use `ESP_LOGx` tags: "IR_TX", "IR_RX", "NOW", "GAME" for easy debug filtering.

  ## Collaboration Rules – Proposal Mode
- **Do not modify files immediately.** If anything is unclear, **ask clarifying questions first**.
- **Propose before changing.** Respond with:
  1) **Questions** (if needed, max 5 targeted),
  2) **Plan** (minimal bullet list),
  3) **Diff (dry-run)** as a unified diff in triple backticks — **do not apply**,
  4) **Rollback notes** (how to undo),
  5) **Tests** (build + runtime checks).

- **Wait for explicit approval.** I will reply **“APPROVE PLAN”** and/or **“APPLY DIFF”** before you make any edits.
- **Scope control.** Only touch files I explicitly name. If other files are required, list them under **Proposed Additional Files** and wait for approval.
- **No refactors without approval.** Don’t rename/move files, change public APIs, reformat unrelated code, or alter task priorities/pin maps.
- **Version target.** Code must compile on **ESP-IDF v5.3.3** only; avoid newer structs/APIs.
- **Style & constraints.** No Arduino APIs; no delay-based timing; keep game logic separate from hardware drivers; use LEDC/GPTimer/RMT as specified elsewhere in this doc.
