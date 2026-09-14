# Task: Implement MAR Yawn-Detection Alert Logic (FatigueSense)

## Context
This is part of the FatigueSense driver fatigue detection system. Facial landmarks are extracted from camera frames, and the Mouth Aspect Ratio (MAR) is computed per frame to detect yawning as one of two independent fatigue triggers (the other being EAR/eye closure).

## Goal
Implement the yawn-detection and alert-triggering logic for MAR, matching the behavior specified below, and wire it to the existing buzzer/LED alert mechanism.

## Specification

**MAR calculation:**
```
MAR = |Top - Bottom| / |Left_Corner - Right_Corner|
```
(inputs are mouth landmark coordinates already available from the landmark model)

**Yawn event detection:**
- A single frame counts as a "yawn event" when `MAR > 0.60`.

**Alert trigger condition:**
- Maintain a rolling 60-second window of detected yawn events.
- Trigger a full fatigue alert only when **3 or more yawn events** occur within that rolling 60-second window.
- A single MAR spike above 0.60 should NOT by itself trigger an alert — only the 3-in-60s pattern should.

**Alert action (on trigger):**
- Send an HTTP alert command (existing endpoint: lightweight HTTP server on port 80) to activate the physical buzzer and LED via GPIO.
- This should reuse the same alert-trigger pathway used by the EAR (eye closure) detection logic, since MAR and EAR are independent triggers into the same alert mechanism.

## Requirements
1. Implement a rolling-window counter (sliding window or timestamp-queue approach) for yawn events — do not use a simple fixed-bucket reset, since the window must be truly rolling/continuous.
2. Debounce/de-duplicate so that sustained MAR > 0.60 across consecutive frames during a single yawn is not double-counted as multiple yawn events (define a reasonable minimum gap or state-transition rule, e.g. count on the rising edge of MAR crossing 0.60, not every frame while above it).
3. On alert trigger, fire the same HTTP call used by the existing EAR-based alert pipeline.
4. Reset/clear expired events from the window as time passes (older than 60s).
5. Add logging for: each yawn event detected (timestamp, MAR value), and each full alert triggered (timestamp, number of events in window).

## Acceptance Criteria / Test Cases
- [ ] MAR briefly exceeds 0.60 once in a 60s window → no alert fires.
- [ ] MAR exceeds 0.60 in 3 distinct yawns within a 60s window → alert fires on the 3rd event.
- [ ] 3 yawns occur, but spread across more than 60 seconds (e.g., 2 within window, 1 outside) → no alert.
- [ ] After an alert fires, the system continues to track new yawn events correctly (window logic doesn't get stuck).
- [ ] A single sustained mouth-open state (MAR staying above 0.60 for several consecutive frames) counts as exactly one yawn event, not several.
- [ ] End-to-end latency from the 3rd yawn event to buzzer/LED activation is measured and reported.

## Out of Scope
- EAR (eye closure) detection logic — already implemented separately.
- Landmark extraction / MAR input coordinates — assume these are provided by the existing model pipeline.
