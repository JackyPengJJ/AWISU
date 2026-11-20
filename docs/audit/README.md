# Design closure and assignment checks

This record accompanies the revised EE5110 technical report. The assignment calls for a surveyed and justified UAV design; the simulation is supplementary evidence. The original 2.65 kg experiment remains byte-for-byte preserved. New component tests are stored separately in `results/candidate18`.

## Assignment traceability

| Requirement | Design evidence | Verification scope |
| --- | --- | --- |
| Mass strictly below 3 kg | 2901.6 g installed BOM including 100 g contingency | Mass allocation and matched rigid body; no physical weighing |
| 80 × 2.5 × 8 m aisle | Compiled rack geometry, clearance and ceiling checks | `scene_audit.json`; original and candidate geometries kept distinct |
| Five equally spaced levels, both sides | 0.8/2.4/4.0/5.6/7.2 m; 200 labels per side | Original 400 cargo units retained |
| Aisle-facing barcodes | Original 100 × 44 mm short-code labels | Native candidate-camera exposure pairs; denser codes explicitly tested analytically |
| GPS-denied unknown environment | FAST-LIVO2 integration, local map, free-space stopping backup, rack discovery and coverage/return gates | Technical design in Sections 3.2–3.5; no integrated unknown-layout success claimed |
| One takeoff, no battery change | Same 6S2P pack; all scan, turn, recovery and return actions budgeted | Loaded-curve model, explicit reserve and rejection boundaries |
| Platform, control and navigation comparisons | Sections 2–4 and primary-source register | Source conditions and engineering allocations distinguished |
| A4, TNR 11.5, ≤20 pages | XeLaTeX authority; chosen figure typography preserved | Export validation and page review |

The task does not specify barcode dimensions, encoding length, cargo count, temperature, cutoff or reserve percentage. These are design inputs, not assignment facts. Nominal launch/end staging space must be observed free; it is an installation condition beyond the specified rack dimensions.

## Optical decision

The selected design retains BFS-U3-122S6M-C global-shutter heads, 12 mm lenses and y=±0.40 m optical centres. Bodies are 29×29×30 mm; lens external envelope is Ø42×36.2 mm. Mechanical principal-plane placement and servo settling require calibration. A same-mass 20 MP alternative is recorded, but its rolling/global-reset exposure model does not match the analysed global-shutter model.

The original 100 mm short payload has 99 modules including quiet zones. The 12-character comparison has 187. Finite-label containment covers the bounded camera/rack/label uncertainty box; pose-specific depth and ray-ratio bounds give a tighter projection Jacobian than using total ray length.

| Payload | Width (mm) | Minimum pixels/module | 2.5 target |
| --- | ---: | ---: | --- |
| W0000 (99 modules) | 100 | 2.563 | Pass |
| W0000 (99 modules) | 200 | 5.070 | Pass |
| W0000 (99 modules) | 250 | 6.303 | Pass |
| INVENTORYABC (187 modules) | 100 | 1.357 | Outside envelope |
| INVENTORYABC (187 modules) | 200 | 2.684 | Pass |
| INVENTORYABC (187 modules) | 250 | 3.337 | Pass |

All label corners must remain in the eroded frustum; a reduced sampling guarantee never marks an unreadable case complete. A 300-case numerical rotation check tests the derivative bound and finite corners. Fourteen native 4096×3000 exposure pairs in the warehouse decode the expected unchanged labels in both views. Fifty-six perturbations of located crops retain the expected code under the specified 0.75-pixel blur, brightness and noise model. These are bounded optical/component tests, not a measured warehouse failure probability.

## Energy and current decision

The 19.8 V cutoff is rejected by the loaded-curve audit. The same pack is screened at 18.0 V. The quasi-static model solves P=IV, V=6v_cell(q,I/2)−IR and dq/dt=I/(2×3600). Cell curves already include cell internal drop; only the separate assembly connection resistance is added. Source PDF hash, curve axes and digitised samples are retained in this directory.

The selected 23°C case uses −40 mV/cell, capacity-axis scale 3000/3120, 10 mΩ connections and a 30 A shared limit. These are declared sensitivities/assembly targets, not certified pack properties. Remaining energy is evaluated at the maximum phase-average adverse load (440.38 W).

| Recoveries | Mission (s) | Mission (Wh) | Reserve at 10 mΩ | Reserve at 20 mΩ |
| ---: | ---: | ---: | ---: | ---: |
| 2 | 669.5 | 73.63 | 26.65% | 23.89% |
| 8 | 731.1 | 80.35 | 20.00% | 17.00% |
| 20 | 854.3 | 93.78 | 6.72% | 3.23% |

The two-recovery case delivers about 100.39 Wh under its mission-plus-tail history. Five turns are charged: four between levels and one before forward return. Eight recoveries sit at approximately 20% reserve and are treated as marginal; the discretised conservative screen admits at most 7 with the selected inputs. This is not an empirically established retry limit. The 72-case sweep, load-order checks and aged/connection-loss failures remain available. At 90% capacity scale and 20 mΩ, two recoveries leave 18.7%, below the requirement.

A low-voltage screen using the manufacturer 22.2 V thrust–power samples gives collective T/W 1.298 and single-axis roll/pitch torque up to 2.008 N m at hover collective. A 20% differential produces 1.207 N m at 24.26 A. Voltage-squared thrust scaling, installation loss and the yaw coefficient are engineering models; a physical low-voltage map is a qualification item.

## Matched plant and preserved experiment

The new 2.9016 kg plant uses 600 mm motor diagonal, the full installed mass-distribution matrix, 1.00 m diameter and 0.50 m collision height. The component ledger gives Jxx≈0.1026, Jyy≈0.0534 and Jzz≈0.1330 kg m². Three 32 s manoeuvre runs retain full angular dynamics, lag and shared-current allocation, with delayed/noisy position/velocity and exact attitude feedback. Peak modelled current is 25.48 A, maximum sampled position error 0.0787 m and braking distance to <0.05 m/s is 0.527–0.531 m.

A complete matched known-route schedule takes 669.5 s, including five turns and two declared recovery injections. RMSE is 0.0138 m, maximum sampled error 0.101 m and all-step peak model current 25.84 A. Nominal wall/ceiling clearance is at least 0.487 m. Its trace includes one extra second of terminal observation; operational timing ends at the scheduled landing. Physical execution, prescribed route and proxy-state assumptions define this evidence.

The preserved 2.65 kg plant, fixed cameras, reference mesh, controller selection, 501.02 s route and 400-ID evidence remain separate. The new static pitch frames do not assert servo timing, and the new manoeuvre and full-route execution tests do not assert SLAM or a complete decoded inventory flight. The report labels each evidence source. No new training was performed and no historical records were rewritten.

## Remaining physical and integration scope

Before hardware operation, measure assembled pack delivery/resistance and cell imbalance, low-voltage thrust/current, yaw torque, loaded servo settling, optical contrast/distortion and target-computer latency. Unknown-layout estimator/coverage integration is designed but not demonstrated end-to-end. These are explicit release conditions; the technical report’s required design topics and supplementary simulation evidence are not represented as a manufactured aircraft.

## Reproduction

Run from the repository root with the project environment:

```bash
python -m segd.battery_model --out runs/candidate18
python scripts/check_candidate18_boundary.py --out runs/candidate18
MUJOCO_GL=osmesa python -m segd.candidate_control --out runs/candidate18
MUJOCO_GL=osmesa python -m segd.matched_mission --out runs/candidate18/full_route
python -m segd.matched_optics --out runs/candidate18
MUJOCO_GL=osmesa python -m segd.matched_render --out runs/candidate18/optical_render
python -c "from segd.matched_render import degradation_study; degradation_study('runs/candidate18/optical_render')"
MUJOCO_GL=osmesa python scripts/audit_candidate_scene.py --out runs/candidate18
python -m unittest discover -s tests -v
python scripts/verify_release.py
```
