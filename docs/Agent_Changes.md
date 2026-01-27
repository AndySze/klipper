# Agent Changes

## 2026-01-27: Klipper-Go MCU stability + temperature bring-up

- **What changed:** Improved MCU handshake robustness, shutdown handling, and ADC temperature sampling/decoding; reduced configuration-time serial traffic by pausing `get_clock` polling and pacing legacy sends.
- **Why:** On real AVR hardware the MCU could enter shutdown shortly after `finalize_config`, preventing `analog_in_state` updates and leaving temperatures at `0.0`.
- **Impact/Risk:** Low-to-moderate. Changes affect real-hardware comms paths (`hosth4`) and protocol parsing; may alter timing/throughput on some setups. Serialqueue mode is unchanged; legacy mode gains optional pacing.
- **Validation:** Code-level review and trace-driven debugging. Full hardware validation requires rebuilding `go/cmd/klipper-go` and confirming no post-`finalize_config` shutdown and that `analog_in_state` updates arrive.

