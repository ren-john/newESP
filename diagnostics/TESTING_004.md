# Testing 004: find the ESP memory fault

Testing 003 still rebooted with StoreProhibited at address 0x2d in __ssprint_r, called by snprintf inside httpd_resp_send_chunk. The failing ELF SHA-256 starts af2f26d31 and exactly matches the preserved ELF in diagnostics/testing003-af2f26d31. This establishes that the previous mitigation did not eliminate the ESP crash. It does not establish that snprintf itself is defective.

The new diagnostic build adds light heap poisoning, strong compiler stack protection, the FreeRTOS end-of-stack watchpoint, and heap checks before/after camera acquisition and HTTP sending every 30 frame attempts. Progress logs report stream stack headroom and free internal memory every five seconds. These checks add overhead and are intended for diagnosis.

This build retains the Testing 003 camera settings and Android application. It does not change the model or remove HTTP chunk formatting to conceal the failing call site.

## Procedure

1. Flash using the project's normal ESP-IDF environment: `idf.py -p COM9 flash monitor`.
2. Confirm the boot log contains `DIAG004` and `PSRAM DMA mode disabled`.
3. Close browser stream tabs and run the installed Android app as before.
4. Save the complete serial output from boot through the FIRST error and reboot. Include any CORRUPT HEAP, stack smashing, stack watchpoint, or DIAG004 progress messages immediately before the panic.
5. If it runs without failure, continue at least 15 minutes and record that duration. A successful diagnostic run may reflect changed timing and does not alone prove the defect fixed.

Heap corruption detected at a checkpoint only narrows the time interval: another task or DMA may have written the bad memory. If guards remain intact while formatted output still crashes, next isolate PSRAM code/read-only-data relocation and inspect the matching core dump rather than assuming model load or increasing stack sizes again.

Reference: https://docs.espressif.com/projects/esp-idf/en/v5.4.4/esp32s3/api-reference/system/heap_debug.html
