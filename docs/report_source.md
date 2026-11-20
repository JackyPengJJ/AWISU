# AWISU: Autonomous Warehouse Inventory with a Small UAV

Junjie Peng

[Code](https://github.com/JackyPengJJ/AWISU) · [Video](https://youtu.be/ZQHlchVMqs8)

## Abstract

We present AWISU, a small-UAV design for bilateral barcode inventory in an unknown, GPS-denied warehouse aisle. The design combines LiDAR-inertial-visual navigation, two pitch-adjustable global-shutter cameras, a spatial inventory record and immediate local recovery. Complete-label projection constrains continuous scanning to 0.85 m/s for the supported label geometry. Installed mass is {{mass_g:.1f}} g including 100 g contingency; individual flight and recovery actions determine mission duration and energy. A 400-item scenario with two recoveries takes {{mission_s:.1f}} s and requires {{adverse_energy_wh:.2f}} Wh under adverse power assumptions, or {{required_delivered_wh:.2f}} Wh delivered while retaining a 20% reserve. A MuJoCo study evaluates reference tracking and image decoding. The selected residual controller reduces full-route RMSE from 0.1148 to 0.1038 m; crop verification recovers 400 item IDs from 1420 images. Optical sampling, coverage and recovery workload jointly determine the operating envelope.

## 1. Introduction

Warehouse inventory requires both readable barcodes and spatial coverage. Each observation must identify a physical cargo instance; empty, unseen and unresolved locations need distinct records. A clearer image may require a closer viewpoint, consuming energy needed to finish the aisle.

The assignment specifies an 80×2.5×8 m aisle, five equally spaced cargo levels on each side, aisle-facing barcodes, mass below 3 kg and one-flight completion. Rack poses, obstacles and cargo locations are unknown. These constraints couple platform choice, autonomous control and GPS-free navigation.

AWISU combines bilateral pitch heads, a continuous five-level snake and immediate local recovery. Figure 1 connects complete-label visibility, motion admission and an action-based energy budget. The worked scenario retains 400 cargo units and the reference scene’s 100×44 mm short-code labels; neither quantity nor label size is prescribed by the assignment. The aisle is static and access-controlled. Label density, connection loss and recovery workload are varied; battery calculations use 23 °C source curves.

![Figure 1. AWISU overview. Recorded images show acquisition; coverage states and recovery illustrate the proposed task logic. Solid and dashed paths denote covered and planned travel.](figures/framework.pdf)

## 2. Related Work and Design Choices

### 2.1. Inventory workflow and aircraft platform

Corvus One [3] and Verity [4] organise inventory around repeated observations, location association and warehouse records. We adopt this organisation: each confirmed item retains image evidence and a spatial identity, while failed reads remain actionable records. Commercial workflows motivate the task structure; component specifications and mission calculations determine aircraft sizing.

Platform alternatives include a documented development kit, a custom quadrotor and a hexarotor. Holybro's X500 v2 uses a 500 mm frame, 2216 KV920 motors, 4S ESCs and 1045 propellers [5]. We select a 600 mm custom frame to accommodate protected 14-inch rotors. Its larger disc area reduces loading, at the expense of aisle clearance and custom structure. A hexarotor adds actuators and protection area within the same mass limit.

Table 0. Platform alternatives for the warehouse task.

| Platform | Useful capability | Limiting trade-off |
| --- | --- | --- |
| Commercial [3,4] | Integrated inventory workflow | Component detail unavailable |
| X500 v2 [5] | Documented PX4 kit | Stock 4S / 10-inch propulsion |
| Custom quadrotor | 600 mm; 14-inch rotors | Custom structure; selected |
| Hexarotor | More actuators | Added mass and guarded area |

### 2.2. Navigation in repetitive aisles

Repeated uprights weaken along-aisle observability; plain cartons weaken image tracking. ORB-SLAM3 uses images/IMU [20], FAST-LIO2 uses LiDAR/IMU [19], and FAST-LIVO2 fuses geometric and photometric residuals [21]. The latter is selected for complementary cues, accepting calibration/timing work. FAST-LIO2 supplies a geometry-only fallback.

Table 0. Navigation alternatives and warehouse trade-offs.

| Method | Measurements | Warehouse trade-off |
| --- | --- | --- |
| ORB-SLAM3 [20] | Images + IMU | Low mass; texture dependence |
| FAST-LIO2 [19] | LiDAR + IMU | Geometry; axial degeneracy |
| FAST-LIVO2 [21] | LiDAR + images + IMU | Complementary cues; calibration |

### 2.3. Control and safe trajectory execution

PX4’s cascaded position/velocity and attitude/rate loops are selected for firmware support and low computation [42], with explicit tilt, thrust and shared-current limits. Table 3 compares geometric feedback [40], LQR [45] and MPC [41]. A bounded learned residual is evaluated separately [28,29].

Table 0. Controller comparison for narrow-aisle flight.

| Controller | Constraint handling | Selection trade-off |
| --- | --- | --- |
| Cascaded PID [42] | Saturation / anti-windup | PX4 implementation; selected |
| Geometric [40] | Added actuator constraints | Nonlinear attitude; inertia model |
| LQR [45] | Added saturation checks | Local model; low compute |
| MPC [41] | Predicted state/input limits | Solver deadline; model sensitivity |

Planning adopts FASTER’s observed-free stopping backup [22]. Short local segments and inventory frontiers fit the optical speed limit; a late update leaves the accepted stopping trajectory executable.

## 3. System Design and Method

### 3.1. Hardware and installation

We select four T-MOTOR MN3508 KV380 motors, P14×4.8 carbon propellers and AIR40A 2–6S ESCs [6–8]. GEPRC specifies its 6S2P VTC6 pack at 6000 mAh, 607 g and 30 A continuous discharge [37]; we allocate 627 g installed. A plastic-case Pixhawk 6C and analog PM02 V3 provide flight control and voltage/current monitoring [10,11]. MID-360 supplies LiDAR/IMU observations, OAK-D Lite supplies forward stereo and navigation images, and an Orin Nano 8 GB module runs on a Hiwonder Mini carrier [12–15]. The 180 g computer allocation includes the 77 g carrier, module, cooling, storage and supports.

Each side head uses a BFS-U3-122S6M-C: 4096×3000 pixels, 3.45 μm pitch, global shutter, 36 g and 3 W maximum [35]. A 12 mm RT-V1228-MPY2 lens covers the 1.1-inch sensor [36]. Its 110 g installed allocation covers the 97 g catalogue lens, mounting and variation. Two pitch assemblies receive 70 g for servos, balance and supports [18]. Figure 2 links a reference mesh to these component roles; opposing cameras observe both racks during translation.

![Figure 2. Reference aircraft and selected components. Render: Skydio X2 [30]; product photos: T-MOTOR [6], Holybro [10], Opto Engineering [36]. Callouts identify proposed component roles.](figures/hardware.pdf)

We reject the 5 MP/8 mm option because its 55 g lenses reduce mass but require approximately 1.61 mm barcode modules under the baseline uncertainty envelope [16,17]. Our 12.3 MP installation adds 130 g in lenses/supports for improved sampling. With these optics, the 755 g 5 Ah LiPo exceeds the mass limit [9]. Flywoo's 596.2 g P30B pack saves 10.8 g, but its published 35 A description leaves continuous pack current unspecified [39]. We accept the GEPRC mass penalty for its documented continuous rating. Amprius's 465 g option supplies 12 A continuous, below the approximately 20 A low-voltage mission demand [44]. A same-mass 20 MP BFS-U3-200S6 offers 2.4 μm pixels but uses rolling shutter with global reset [46]; it requires a different exposure model. The global-shutter head is retained for the analysed moving acquisition.

Table 0. Installed mass budget.

{{MASS_TABLE}}

The {{mass_g:.1f}} g total includes 100 g contingency and leaves {{mass_margin_g:.1f}} g to the limit. Custom structure uses two 180×140×2 mm carbon plates and four 16/14 mm outer/inner-diameter tubes, approximately 240 mm long. At 1600 kg/m³ these contribute 161 g and 72 g; the remainder of the 340 g frame allocation covers joints, fasteners and landing supports. Rings/bracing receive a separate 140 g. Substitutions must fit installed allocations, including cables and supports.

Adjacent motors are $0.600/\sqrt{2}=0.424$ m apart. With 0.3556 m rotors and two 0.020 m radial protection allowances, protected discs remain {{propeller_gap_m:.3f}} m apart. Rounding the {{geometry_outer_diameter_m:.3f}} m enclosing diameter gives a 1.00 m collision envelope of 0.50 m height. Side optical centres lie at $y=\pm0.40$ m, level with the body reference, with principal rays outside the rotors. Battery placement below the plates balances the elevated LiDAR and outboard optics.

Bank changes the swept envelope. For radius $r=0.50$ m and half-height $h=0.25$ m, lateral extent is $r\cos\theta+h\sin\theta$. At 12°, subtracting that extent and tracking/localisation/map allowances of 0.15/0.15/0.05 m from the 1.25 m half-aisle leaves {{side_clearance_budget_m:.3f}} m. At the 7.2 m upper scan height, vertical extent is $r\sin\theta+h\cos\theta$; allowances of 0.10/0.075/0.025 m leave {{ceiling_clearance_budget_m:.3f}} m ceiling clearance. Scan admission requires lateral sigma ≤0.05 m and vertical sigma ≤0.025 m; rack protrusions are checked against the same tilted envelope.

### 3.2. Power, interfaces and computation

A fused 6S branch feeds the ESCs; PM02 V3 powers Pixhawk separately. D36V50F12 regulates the computer, LiDAR and LEDs to 12 V; D36V50F6 supplies an isolated 6 V servo rail [11,33,34]. Regulation keeps the carrier below its 19 V maximum at 25.2 V full pack charge. The electrical operating range is 18.0–25.2 V. The 60 W auxiliary allocation covers computing, sensors, lighting and regulators; converter thermal/current curves determine continuous branch admission.

MID-360 connects through Livox driver2 over Ethernet, preserving per-point acquisition offsets for deskewing. OAK-D Lite supplies calibrated stereo/depth and grayscale images over USB. We specify an adapter for the image model, intrinsics, exposure timestamps and extrinsics required by FAST-LIVO2 [21,43]. Integration measures clock offset/drift against a 1 ms alignment target. At 1 m/s that offset contributes 1 mm translation; a 0.2° camera–LiDAR rotation error contributes about 17 mm at 5 m, so rotational calibration dominates this example.

The estimator interface registers its body frame to Pixhawk FRD and converts local output to NED for MAVLink ODOMETRY, including covariance, resets and acquisition timestamps [23]. PX4 receives smooth local odometry and 50 Hz setpoints over 921600-baud UART. A 250-byte odometry message at 50 Hz uses 12.5 kB/s; our 20 kB/s control/telemetry allocation fits approximately 92 kB/s before framing overhead. Stabilisation and the stopping backup run onboard independently of inventory and ground-link queues [24].

Table 0. Input scale and bounded resource allocations.

| Function | Input / allocation | Deadline or overload action |
| --- | --- | --- |
| Estimation | 200 Hz IMU; 10 Hz LiDAR; 30 Hz images | 50 Hz output; stop scan after 0.15 s correction age |
| Map / planner | 10 Hz map; 5 Hz plan; 0.10 m grid | Retain known-free stopping backup |
| Side images | Two 4096×3000 Mono8; 10 fps each | 245.76 MB/s; decode native crops |
| Image queues | Two frames per camera; 49.15 MB | Bound queues; pause missed admission |
| Occupancy | 30×5×10 m; 0.10 m cells | 24 MB cells; 256 MB with indices/history |
| Application | 512 MB map; 128 MB buffers | 3.5 GB application; about 2 GB OS |

LiDAR adds 4.8 MB/s (200k points/s, 24 bytes/point); 640×400 stereo adds 15.36 MB/s at 30 Hz, totalling about 266 MB/s. Pair bursts reach 20 fps/side. Staggered transfers, bounded queues and native-crop decoding limit load. Four-frame bursts contain 49.15 MB. At a required 400 MB/s usable USB payload rate, reserving 15.36 MB/s for stereo clears the burst in 128 ms; the second pair arrives 50 ms later, leaving 22 ms of its 100 ms deadline for crop processing. These are admission targets for the 15 W mode; overload pauses coverage. A 64 GB drive stores records and selected evidence.

### 3.3. Dynamics and flight control

We model position $\boldsymbol p$, velocity $\boldsymbol v$, body-to-world attitude $R$, body rate $\boldsymbol\omega$, total thrust $f$ and moment $\boldsymbol\tau$ in an upward-positive local frame [40]:

$$ \begin{aligned}\dot{\boldsymbol p}&=\boldsymbol v,\quad m\dot{\boldsymbol v}=Rf\boldsymbol e_3-mg\boldsymbol e_3+\boldsymbol d_f,\\ \dot R&=R[\boldsymbol\omega]_\times,\quad J\dot{\boldsymbol\omega}=\boldsymbol\tau-\boldsymbol\omega\times J\boldsymbol\omega+\boldsymbol d_\tau.\end{aligned} $$

The controller interface converts this convention to NED/FRD. Rotor thrust and reaction torque follow $f_i=k_T\Omega_i^2$ and $q_i=k_Q\Omega_i^2$, with first-order motor lag. Our reference simulation uses 40 ms lag and 200 Hz integration. The deployment cascade tracks position, velocity and acceleration, then computes orientation and allocates moments through PX4's attitude/rate loops [42].

Position/velocity feedback generates a desired acceleration, with clamped integral correction. Reference horizontal gains 2.5 $\mathrm{s}^{-2}$/2.6 $\mathrm{s}^{-1}$ give a 1.58 rad/s natural frequency and 0.82 damping ratio; vertical gains 4.0/3.2 give 2.0 rad/s and 0.80. A 2 Hz attitude target separates the inner and outer loops. Saturation and invalid estimates freeze integration; loaded response measurements determine deployment gains.

Motor/ESC/propeller groups contribute approximately 0.026 kg m² roll/pitch inertia at 0.212 m offsets. A component placement ledger and the parallel-axis theorem give $J_{xx}=0.1026$, $J_{yy}=0.0534$ and $J_{zz}=0.1330$ kg m². Lateral cameras increase roll/yaw inertia substantially. The full matrix, including products of inertia, defines the matched validation plant; these are installation estimates rather than CAD measurements. Current-aware allocation takes the form of weighted wrench fitting:

$$ \boldsymbol f^\star=\arg\min_{0\le f_i\le f_{max}(V)}\|W(B\boldsymbol f-\boldsymbol w_c)\|_2^2\quad\mathrm{s.t.}\quad\sum_i P_i(f_i,V)+P_{aux}\le VI_{max}. $$

Here $B$ maps thrusts to wrench, $\boldsymbol w_c$ is the demand, and $W$ prioritises collective and roll/pitch before yaw. The matched test implements this priority with bounded scaling and checks every command against the 30 A shared limit. Its power model uses the measured 22.2 V map with explicit installation and low-voltage screening factors; the retained reference run uses inverse allocation with per-rotor clipping. At 0.85 m/s, 0.10 s latency and 0.6 m/s² braking imply 0.687 m stopping distance. A 1.5 m observed corridor includes mapping/tracking margins; a shorter corridor reduces admissible speed.

### 3.4. State estimation and local geometry

FAST-LIVO2 propagates position, velocity, attitude and inertial biases, then applies timestamped LiDAR/image corrections [21,43]. PX4 retains inertial stabilisation. Exposure, clock, distortion and extrinsic calibration share one transformation chain. Information conditioning, correspondence count, innovations and visual consistency jointly detect repetitive-bay ambiguity. Weak along-aisle information requests a rack-end or cross-bracing view. Map corrections update a map-to-odometry transform, preserving control-frame continuity.

Our local voxel map distinguishes occupied, observed-free and unknown space, following OctoMap [25]. Planning uses a 0.10 m grid with underlying-point queries near obstacles; pruning preserves the return corridor and rack locations. Initialisation aligns gravity, defines an origin and fits floor/parallel rack planes. Upright clusters define bays; support and cargo surfaces define the task's five height bands. Observed surfaces define scan regions.

Initial geometry thresholds are RANSAC residual ≤max(0.03 m, 3σ_range), five consistent 10 Hz plane updates, <5° normal disagreement and 0.10 m upright/height clustering. These are initial tuning settings. Ambiguous bands pause association and request another view. Endpoint confirmation requires observed rack termination and a free inflated turn volume. Spatial association combines a 3σ gate, projected face overlap and side/band identity; ambiguous matches remain tentative. Empty slots require observed free faces.

MID-360's −7° to +52° elevation range leaves a lower blind region [12]. We mount forward stereo approximately 15° downward and yaw 180° at each endpoint so it faces the next pass. Four inter-level turns plus a fifth before return cost 20 s. Keeping heading would save this time but require rear sensing or stricter LiDAR-only admission. Climb/descent requires recently observed free columns and ceiling clearance at the highest band. Figure 3 quantifies clearance and nominal ray limits; installed geometry supplies additional body/rotor occlusion checks.

![Figure 3. Motion limits and sensing envelope. Clearance includes tracking, localisation and map margins; the lower blind zone requires a previously mapped descent column.](figures/04_observability.pdf)

### 3.5. Coverage, confirmation and local recovery

At 5 Hz, the local planner receives state/covariance, the map, coverage records and remaining energy. It inflates obstacles by the tilted envelope, finds a short path and fits a timed segment with a stopping backup [22]. Collision, speed, acceleration and reserve checks precede replacement of the current plan. A late update leaves the accepted backup executable. A blocked endpoint selects another mapped transition column; otherwise we return with a blocked-region record.

Each cargo record uses side, bay/band and spatial track as its key, with payload, observation times, covariance and confirmation state as attributes. This preserves distinct instances of repeated SKUs. An action is admitted when:

$$ \mathcal A(a)=\mathcal L\land\mathcal F(a)\land\mathcal O(a)\land\left[E_{rem}\ge E_a+E_{unfinished}+E_{return}+0.20E_{del}\right]. $$

$\mathcal L$ checks current localisation; $\mathcal F$ checks the swept volume and backup; $\mathcal O$ checks image eligibility whenever a surface is marked inspected. Remaining workload $E_{unfinished}$ updates when cargo or blocked space is discovered. Transit alone cannot mark a surface complete.

Initialisation supplies the origin and verified stopping corridor. Local geometry then extends the rack frontier; image/pose gates update spatial cargo tracks. Failed tracks enter immediate recovery, endpoint/free-column checks release the next height band, and coverage plus reserve determines completion or return with exceptions.

Each side carries two off-axis LUXEON Rebel LXML-PWC1-0100 LEDs in a 0.70 A series string driven by TPS92512 from 12 V [34]. A 30° beam covers about 0.16 m² at 0.85 m; 360 lm gives approximately 2 klx before losses, motivating a 0.20 ms exposure. The decoder detects candidates at reduced resolution, reads native crops with zxing-cpp [26], checks payload syntax and associates each crop using its exposure pose and calibrated pitch.

Confirmation requires consistent, quality-checked whole-label reads from two acquisitions of the same spatial track. Disagreement triggers a changed exposure or viewpoint. An unreadable carton remains an unresolved instance; confirmed aisle-visible units are counted per SKU and any encoded quantity is parsed separately. Empty, unseen, inaccessible and undecodable locations retain different states.

A failed read preserves opposite-side confirmations and freezes the unfinished boundary. Recovery starts with an in-view retry, followed by braking, recentering and an optional approach up to 0.10 m towards the failed side. Aimed pitch/height replaces the normal sweep during this settled read. With 0.70–0.85 m range and the adopted errors, the centred zero-pitch view retains a {{targeted_width:.3f}} m safe label-centre interval. A 0.10 m offset leaves 0.259 m lateral clearance against the 0.25 m target and improves centre-ray sampling by about 13%. Smaller free space or larger uncertainty reduces the offset. Both outward and return sweeps must pass collision checks before scanning resumes at the same boundary.

Each failed item receives at most two settled attempts. Completion requires both sides and all five bands inspected, every observed instance confirmed and no unchecked reachable surface boundary. Persistent failures produce an inventory with exceptions; reserve admission selects return when further recovery cannot be afforded.

## 4. Design Analysis

### 4.1. Complete-label visibility and acquisition

The selected optics give $f_x=12/0.00345=3478$ pixels and approximately 61.0° horizontal/46.7° vertical field. The primary validation retains the original 100×44 mm labels. W0000 uses 79 encoded modules plus two 10-module quiet zones: $N_m=99$ and $X=100/99=1.010$ mm. A 12-character Code128-B example has 187 modules, so the same outer width provides only 0.535 mm modules. Width alone therefore does not define readability; no warehouse relabelling is assumed.

Visibility is evaluated at all four label corners against the image frustum. For corner rays $q_k$, inward unit normals $n_j$, camera-orientation uncertainty $\delta$ and finite-label displacement $\epsilon_l$, sufficient containment is:

$$ n_j^T q_k-\|q_k\|\sin\delta-\epsilon_l(1+\sin\delta)\ge0,\quad \epsilon_l=2r_l\sin(\alpha_l/2). $$

The admitted box includes 0.80–0.95 m range, ±50 mm vertical error, ±10 mm longitudinal error, 2° acquisition orientation error, 5° label rotation and a 24-pixel image border. Vertex checks include half a 1 mm height cell. The separate 12° envelope applies to collision checking; scanning resumes after attitude settles into the optical cone.

The three poses cover a complete 1.6 m band. After erosion by the 100×44 mm label and errors, the minimum usable travel interval is {{safe_width:.3f}} m. Each 0.20 s slot reserves 0.14 s for pitch motion/settling and exposes at 0.14/0.19 s. Across cycles, alternating gaps are 0.05/0.55 s. Within constant-speed acquisition, any phase contains two eligible observations when:

$$ v(T+2j)\le W_{safe},\qquad 0.85(0.60+2\times0.002)=0.5134\ \mathrm{m}. $$

The minimum pair separation is 46 ms. This supplies temporal corroboration, not statistical independence. Persistent glare, occlusion or disagreement requests a changed view. The 140 ms settling and 100 ms processing allocations are implementation deadlines; a missed deadline pauses coverage and consumes explicit dwell energy.

For sampling, optical depth must correspond to the chosen pitch. If $q_c=(X_c,Y_c,Z_c)$ and $a$ is a unit module direction in the camera frame, horizontal projection satisfies $du/ds=f_x(a_x-(X_c/Z_c)a_z)/Z_c$. Let $\gamma=7°$, $Z_{max}$ bound depth and $t_x$ bound $|X_c/Z_c|$ over the admitted ray box. Then:

$$ s_{min}=\frac{f_x(\cos\gamma-t_x\sin\gamma)}{Z_{max}},\qquad p_{module}\ge s_{min}X. $$

Depth and ray-ratio bounds include camera rotation and label displacement. Selecting an eligible pitch per height gives at least 2.563 pixels/module for the 100 mm, 99-module case, above the 2.5 target. For 187 modules, 100 mm gives only 1.357; 200 and 250 mm give 2.684 and 3.337. The dense 100 mm case is outside this design envelope. Local approach is attempted only if its measured geometry passes the same test; it is not presumed to rescue every barcode. A 0.20 ms exposure bounds centre-ray translational blur to 0.74 pixel at 0.80 m.

![Figure 4. Whole-label admission: (a) three-pitch coverage on the rack face; (b) corner containment schematic; (c) limiting travel budget.](figures/05_imaging.pdf)

### 4.2. Propulsion and shared-current authority

MN3508 KV380 manufacturer tests at 22.2 V give P14×4.8 samples of 710 g/64.38 W, 1100 g/117.66 W, 1350 g/166.5 W, 1580 g/217.56 W and 1730 g/255.3 W per motor [6]. Our 725.4 g-per-rotor hover lies within the first interval. With thrusts expressed in newtons, interpolation gives 265.94 W for four rotors:

$$ P_h=4\left[P_1+\frac{T_h-T_1}{T_2-T_1}(P_2-P_1)\right],\qquad T_h=mg/4. $$

Installation power multipliers are 1.10 nominal and 1.25 adverse. Maximum thrust is screened by $(V/22.2)^2$ and a 0.85 derating. The power map and the voltage ceiling are distinct assumptions. Below the first 710 g point, its power is retained as a monotonic upper bound rather than extrapolated to zero. Applying the shared current constraint gives:

Table 0. Modelled authority with a 60 W auxiliary load and 30 A limit.

| Bus (V) | Collective T/W | Roll/pitch torque (N m) | 20% differential current (A) |
| ---: | ---: | ---: | ---: |
| 22.2 | 1.541 | 5.629 | 19.67 |
| 19.8 | 1.407 | 3.697 | 22.06 |
| 18.0 | 1.298 | 2.008 | 24.26 |

Torque is the single-axis bound at hover collective; simultaneous roll/pitch/yaw demands share that envelope. A 20% rotor-pair differential supplies 1.207 N m, and steady 12° tilt fits the model at 18 V. The matched rigid-body test below checks combined manoeuvres using the same allocator. The voltage-squared ceiling is an engineering screen, not measured low-voltage thrust data.

![Figure 5. Manufacturer power samples at 22.2 V [6] and installed-mass margin to 3 kg. The diamond marks hover interpolation.](figures/02_propulsion.pdf)

### 4.3. Mission duration and energy

Five bilateral passes require 400 m scan travel; return and launch approach add 84 m horizontally. Nominal launch/end staging volumes extend beyond the rack ends and must be observed free; their availability is an installation condition, not an extra assignment dimension. Scan speed is 0.85 m/s, vertical speed is 0.35 m/s and acceleration is 0.6 m/s². Rest-to-rest travel over distance $d$ at speed limit $v$ takes:

$$ t(d)=\begin{cases}2\sqrt{d/a},&d\le v^2/a,\\ d/v+v/a,&d>v^2/a.\end{cases} $$

Initialisation allocates 5 s to stationary alignment, 3 s to local-plane acquisition and 2 s to health/admission checks. Five turn events (including the return turn) each receive 0.5 s confirmation and 4 s yaw/reacquisition; ten boundary holds each consume one 0.60 s optical cycle. Discovery runs during the first scan. Unresolved geometry pauses the scan and consumes the remaining dwell allowance.

A failed item adds 7.66 s for processing, braking/resumption, recentering and two 0.5 s settled attempts. Optional lateral approach/return adds 2.61 s, including 0.30 s settling. Thus two, eight and twenty recovered items add 20.53, 82.14 and 205.32 s. Recovery counts are workload scenarios.

{{ENERGY_TABLE}}

The phase-power model separates installed hover loss, horizontal tilt/drag and positive climbing work. With installation factor $k$, drag area $C_D A=0.15~\mathrm{m}^2$ and phase duration $t_j$:

$$ E_j=\frac{\left[kP_h(1+(a/g)^2)^{3/4}+P_{aux}+\tfrac12\rho C_D A v^3\right]t_j+mg\max(0,\Delta z_j)/\eta_z}{3600}. $$

Vertical phases omit horizontal terms and use $\eta_z=0.60$; the adverse vertical multiplier is 1.10. Auxiliary load is 55/60 W nominal/adverse, with no descent energy credit. Applying the small tilt increment across a horizontal segment is our modelling approximation; loaded tests identify installation and rack-interaction losses.

VTC6 cell data specify 3.6 V nominal and rate-dependent loaded discharge [38]. The 12-cell nameplate is 129.6 Wh. The original 19.8 V cutoff is rejected: the digitised typical 10 A/cell curve supplies only 87.66 Wh before connection losses, below the {{required_delivered_wh:.2f}} Wh delivered-energy requirement. The revised candidate uses an 18.0 V loaded cutoff, retaining the same pack and mass.

Loaded voltage, current and delivered energy are solved together:

$$ V=6v_c(q,I/2)-IR_b,\quad P=VI,\quad \dot q=I/(2\times3600),\quad E_{bus}=\int VI\,dt/3600. $$

Recorded axis calibration digitises the 23 °C, 10/15 A curves. Voltage is interpolated between them; below 10 A/cell, the 10 A curve bounds rate loss. Cell drop is already included. Assembly resistance adds $IR_b$ drop and $I^2R_b$ loss. Screening uses 10 mΩ, a −40 mV/cell shift and 3000/3120 capacity-axis scale: explicit sensitivities rather than a guaranteed minimum-cell curve.

After the adverse mission, remaining deliverable energy is integrated at the largest phase-average load, 440.38 W. Reserve is $E_{remaining}/(E_{mission}+E_{remaining})$. This produces about {{delivered_two:.1f}} Wh for the two-recovery load history, rather than assuming 100 Wh from nameplate capacity. The 72-case sweep varies cutoff, connection resistance, capacity and recovery count; high-load-first/last ordering changes tested reserve by less than 0.04 percentage points.

![Figure 6. Loaded-pack evidence and recovery reserve at 18 V. Curves use manufacturer cell data with stated capacity and voltage sensitivities.](figures/07_energy.pdf)

Table 0. Recovery workload and delivered-energy reserve; 23 °C model.

{{RETRY_TABLE}}

The nominal qualification envelope admits two recoveries with useful margin; eight is beyond the selected reserve boundary. At 20 mΩ, eight leave only about {{eight_high_resistance:.1f}}% reserve. With 90% capacity-axis scale and 20 mΩ, even two leave about {{aged_two_reserve:.1f}}%. These cases are rejected by the 20% rule. Recovery count is not a measured failure probability: the planner updates unfinished work, return energy and remaining dwell after every discovery. A persistent unreadable item remains an exception, never a completed inventory record. Pack resistance/capacity, individual-cell undervoltage protection, temperature and loaded motor maps determine physical release.

## 5. Experimental Evaluation

### 5.1. Implementation and evaluation conditions

We built a MuJoCo reference environment for flight tracking, continuous multilevel scanning and image-based inventory [27]. We use a Skydio X2 mesh from MuJoCo Menagerie, Poly Haven carton/shelf assets, and project geometry for primary rack supports and pallets [30–32]. The plant integrates free-body dynamics, four thrust sites, actuator lag, bounded control, contacts and bilateral rendered cameras. Table 9 identifies the configuration used for the recorded experiments alongside the proposed hardware.

Table 0. Design and experimental configurations.

| Parameter | Proposed aircraft | Reference experiment |
| --- | --- | --- |
| Installed mass | {{mass_g:.1f}} g allocation | 2.65 kg rigid body |
| Envelope | 1.00 m diameter; 0.50 m height | 0.80 m diameter; 0.29 m height |
| Battery | 6S2P 6 Ah; loaded cutoff 18.0 V | 4S 6 Ah model; 88.8 Wh |
| Propulsion | Matched table + installation factors | 330 W induced-power reference |
| Optics | 4096×3000; 12 mm; pitch sweep | 4096×2160; 45° horizontal; fixed |
| Scan speed | 0.85 m/s optical limit | 1.1 m/s prescribed route |
| Navigation | LiDAR/IMU/image estimator | Delayed/noisy position/velocity; ideal attitude |

The reference run isolates tracking and decoding with a prescribed route and state proxies. The 136 s film replays its 501.02 s flight in 72 s (6.96×), using saved-pose external, side and forward views at 30 fps. The local revision updates the design-analysis card; the linked upload presents the earlier reference film.

### 5.2. Controller selection and complete-route tracking

Physics runs at 200 Hz and control at 50 Hz, with 40 ms motor response and four 13 N actuator limits. A bounded acceleration residual augments position/velocity feedback, following residual reinforcement learning [28]. PPO uses two 128-unit hidden layers [29]. Three seeds each completed one million steps of local sinusoidal tracking with randomised mass, motor response, authority and wind. The mission planner supplies the full serpentine reference.

We compare validation-best and final checkpoints from all three seeds plus the baseline on two selection routes: seeds 701/702 and 0/1 N wind. We then test the selected controller separately on seed 703 with 0.5 N wind.

Table 0. Mean position RMSE across two selection conditions.

{{POLICY_TABLE}}

Table 0. Separate complete-route comparison; one route per controller.

{{FULL_ROUTE_TABLE}}

Seed-2 best achieves the lowest selection RMSE, 0.0977 m against the baseline's 0.1056 m. Full-route RMSE falls by 9.6%, from 0.1148 to 0.1038 m, over 501.02 s with no contact. Figure 7 shows regressions in other checkpoints, making checkpoint selection consequential. The selected full-route score remains slightly above the 0.10 m objective.

![Figure 7. Experimental evidence: all candidate scores, full-route error envelopes (1 s min/max) and a native barcode crop.](figures/08_experiment.pdf)

The recorded trajectory alternates forward/reverse 80 m passes and climbs at the current endpoint. Scan/climb/return boundary clearance reaches a minimum of {{airborne_clearance_m:.3f}} m. The full-run 0.068 m minimum includes launch-surface proximity. Boundary distance and MuJoCo scene-contact detection describe different geometries; online operation queries mapped obstacles using the swept envelope.

### 5.3. Barcode decoding and evidence

We swept label size, exposure, speed and brightness over 720 images, obtaining 439 correct decodes and no incorrect payloads across those mixed conditions. This study motivates joint sampling/exposure selection. Processing 1420 rendered side images from the scene produced 400 correct IDs and two singleton misreads. We re-decoded saved crops using original, resized and Otsu-thresholded pixels; consistency filtering rejected both extra candidates, leaving 400 correct IDs with no omissions. Ground-truth inventory enters only the final scorer.

Crop filtering tests consistency within one saved image. The deployment logic in Section 3.5 instead requires separate acquisition times and local recovery when views disagree. These evaluation conditions determine how the decoding result transfers to online confirmation.

### 5.4. Matched design checks

A separately named plant uses 2.9016 kg, the 600 mm rotor diagonal, full component-derived inertia, 1.00×0.50 m collision envelope and y=±0.40 m camera centres. Camera bodies and lenses follow manufacturer dimensions. Kinematic head transforms reproduce −28/0/+28° views; servo dynamics and loaded settling are not simulated. The matched warehouse retains the 80×2.5×8 m aisle, five levels, both racks and original 100×44 mm labels. Added ceiling/end boundaries test geometry without changing historical records.

Three 32 s manoeuvre checks use delayed/noisy position/velocity, exact attitude, physical angular dynamics and 40 ms lag. They reach 25.48 A peak current and 5.65° tilt, with 0.0787 m maximum error. Braking to <0.05 m/s takes 1.20 s over 0.527–0.531 m; yaw torque uses the reference coefficient.

The matched plant also executes the complete {{matched_duration:.1f}} s known-route schedule in the warehouse, including five turns and two declared recovery injections. RMSE is {{matched_rmse:.4f}} m, maximum error {{matched_error:.3f}} m and peak model current {{matched_current:.2f}} A. Nominal wall/ceiling clearance remains at least {{matched_clearance:.3f}} m. State proxies and scheduled recoveries isolate execution; this run does not evaluate online navigation or in-flight decoding.

Fourteen native 4096×3000 warehouse exposure pairs test both sides, three height levels and all pitch commands. Both images decode the unchanged 100 mm label in every pair. Static snapshots exclude servo motion and distortion. Fifty-six located-crop perturbations (0.75-pixel blur, brightness 1/0.4, noise 3 DN) retain the expected code without extra candidates. Three hundred numerical rotation cases verify projected derivatives and finite corners against the analytical bounds. Views, crops, traces and source hashes are retained.

## 6. Discussion and Conclusion

AWISU addresses platform selection, autonomous control and GPS-free navigation through a joint optical, dynamic and energy budget. Retaining the original 100 mm short-code labels avoids presumed warehouse relabelling. Continuous acquisition is supported within bounded pose errors; dense 100 mm labels require another optical configuration or budgeted recovery.

The 18.0 V candidate retains useful modelled reserve while reducing collective authority to 1.30. Loaded curves expose connection and workload limits; matched route and optical tests connect the design to simulation. Physical pack/motor qualification and combined unknown-layout estimation/coverage remain the next validation stage. The preserved reference run supplies separate controller and decoding evidence.

AI assistance supported source discovery, implementation, calculation checking and drafting. The retained records identify experimental outputs; course AI-use policy and submission declarations require confirmation against the current course instructions.
