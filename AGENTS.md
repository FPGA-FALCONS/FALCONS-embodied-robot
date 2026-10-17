# Canonical repository

- User instruction dated 2026-10-18: only push updates to `HITSZ-FPGA-FALCONS/FALCONS-embodied-robot`. Other repositories are historical/read-only for this workflow.
- Reusable packages are maintained directly under `vendor/verilog-reusable-modules/` here. Do not publish changes back to the old source repository.
- Preserve unrelated local changes and other contributors' work. Application J6 slot mapping is defined by `rtl/rc/ht10a_input.v` and `doc/HT10A接线与验收.md`; standalone module examples use different mappings and must not overwrite it.
- Follow `vendor/ORIGIN.md` for application regression after vendor updates. Distinguish module/standalone-example verification from whole-vehicle hardware validation.
