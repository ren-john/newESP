# Testing 005: reduce camera and radio load

The supplied Testing 004 logs explicitly report brownout and reset_reason=9. The latest run streamed for about ten seconds before brownout, with 5,648 bytes of stream-task stack headroom and no preceding heap-guard failure. Another supplied run browned out during Wi-Fi connection before streaming. These establish voltage-related resets; they do not establish that the earlier StoreProhibited panic has the same cause.

This diagnostic profile uses QVGA 320x240 instead of VGA, at most 10 capture attempts per second, one DRAM frame buffer with CAMERA_GRAB_WHEN_EMPTY, and Wi-Fi TX power 20 quarter-dBm (5 dBm). JPEG quality remains 12 and XCLK remains 20 MHz. Capture pacing occurs before taking a buffer and does not catch up with bursts after slow sends. One-buffer mode reduces continuous capture work; it does not turn off the sensor. Existing brownout, heap and stack protections remain enabled. The previous matching ELF/bin/map/configs are preserved in testing004-e04a3618f.

The previous TX request was 40 quarter-dBm; ESP-IDF documents that values 34–43 map to 34 (8.5 dBm). The new setting is checked and read back into the boot log.

1. Flash from the normal ESP-IDF terminal: `idf.py -p COM9 flash monitor`.
2. Confirm TEST005 at boot and TX power=20 quarter-dBm. DIAG004 checkpoint messages remain intentionally enabled.
3. Keep the phone close to the ESP because reduced TX power reduces range. Connect with the app/browser closed for five minutes first.
4. Run browser streaming alone for 15 minutes. Close its stream tab and browser, then run the Android app alone for 15 minutes. Save serial logs from boot through the first failure. Record alert/buzzer activity and whether failure happens before streaming.
5. If brownout repeats, stop treating reconnect logic as the primary fix: inspect the board supply path, connectors, regulator and attached loads. A different USB source does not rule out a downstream voltage drop. Do not disable brownout detection.

This is a stability diagnostic, not validated fatigue-detection tuning. Lower resolution can reduce landmark accuracy; frame-count-based alert thresholds and PERCLOS windows take longer at lower inference FPS. A successful test will require later accuracy/timing validation and staged restoration of settings. Changing several load settings together cannot identify which individual change helped.
