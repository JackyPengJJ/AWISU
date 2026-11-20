# AWISU: Autonomous Warehouse Inventory with a Small UAV

Junjie Peng

[Code](https://github.com/JackyPengJJ/AWISU) · [Video](https://youtu.be/ZQHlchVMqs8)

## Abstract

We present AWISU, a small-UAV design for bilateral barcode inventory in an unknown, GPS-denied warehouse aisle. The design combines LiDAR-inertial-visual navigation, two pitch-adjustable global-shutter cameras, a spatial inventory record and immediate local recovery. Complete-label projection constrains continuous scanning to 0.85 m/s for the supported label geometry. Installed mass is {{mass_g:.1f}} g including 100 g contingency; individual flight and recovery actions determine mission duration and energy. A 400-item scenario with two recoveries takes {{mission_s:.1f}} s and requires {{adverse_energy_wh:.2f}} Wh under adverse power assumptions, or {{required_delivered_wh:.2f}} Wh delivered while retaining a 20% reserve. A MuJoCo study evaluates reference tracking and image decoding. The selected residual controller reduces full-route RMSE from 0.1148 to 0.1038 m; crop verification recovers 400 item IDs from 1420 images. Optical sampling, coverage and recovery workload jointly determine the operating envelope.

## 1. Introduction

Warehouse inventory links barcode reading to spatial coverage and cargo identity. An aircraft must associate each observation with a physical cargo instance, distinguish an empty location from an unseen one, and return with a record of what remains unresolved. In a narrow aisle, these decisions also affect flight: a clearer image may require slower motion or a closer viewpoint, while every retry consumes the energy needed to finish the remaining racks.

The task concerns an 80 m × 2.5 m × 8 m aisle, five cargo levels on each side, aisle-facing barcodes and a take-off mass strictly below 3 kg. Specified dimensions supply mission priors; rack poses, obstacles and cargo locations must be discovered online. Completing both sides in one take-off, without a battery change, couples platform selection, GPS-free navigation and flight control to the coverage task.

AWISU uses separate left and right scanning heads to observe both racks during each longitudinal pass. Pitch sweeps cover a cargo band, and endpoint climbs connect five passes into a continuous snake. Component selection balances installed mass, optical sampling, current and energy. A spatial record preserves confirmed items during local recovery; inspected surfaces and confirmed cargo tracks define completion.

Figure 1 links these choices through a finite-label visibility model, a motion-admission rule and an action-based mission budget. Their outputs set acquisition cadence, admissible motion and recovery workload. Reference tracking and rendered-image decoding experiments then examine the control and imaging portions of the design.

Sizing assumes a static, access-controlled aisle at 5–35 °C. Item count, label geometry and recovery frequency remain design variables. The worked example uses 400 visible cargo units and 250×40 mm labels with unchanged payloads; two, eight and twenty recovery cases show how workload changes one-flight feasibility.

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

Repeated uprights constrain depth but can leave along-aisle displacement ambiguous; plain cartons and changing light also weaken image tracking. Complementary measurements address these two failure modes. ORB-SLAM3 offers visual-inertial estimation and map reuse [20]. FAST-LIO2 registers raw LiDAR points efficiently [19]. FAST-LIVO2 combines geometric and photometric updates in a common estimator [21]. We select the latter to exploit both structural and visual cues, accepting additional calibration and timing work. A geometry-only FAST-LIO2 path supplies a fallback when image quality falls.

Table 0. Navigation alternatives and warehouse trade-offs.

| Method | Measurements | Warehouse trade-off |
| --- | --- | --- |
| ORB-SLAM3 [20] | Images + IMU | Low mass; texture dependence |
| FAST-LIO2 [19] | LiDAR + IMU | Geometry; axial degeneracy |
| FAST-LIVO2 [21] | LiDAR + images + IMU | Complementary cues; calibration |

### 2.3. Control and safe trajectory execution

Cascaded PID, geometric feedback, LQR and MPC offer different ways to track the same low-speed route. Geometric control handles nonlinear attitude directly [40]; LQR gives a local model-based reference [45]; MPC incorporates predicted constraints at a greater modelling and solver cost [41]. We select PX4's cascaded position/velocity and attitude/rate structure [42], with explicit tilt, actuator and shared-current limits. Its established firmware implementation avoids adding an optimisation deadline to the companion computer. We evaluate a bounded learned residual as a supplementary tracking experiment [28,29].

Table 0. Controller comparison for narrow-aisle flight.

| Controller | Constraint handling | Selection trade-off |
| --- | --- | --- |
| Cascaded PID [42] | Saturation / anti-windup | PX4 implementation; selected |
| Geometric [40] | Added actuator constraints | Nonlinear attitude; inertia model |
| LQR [45] | Added saturation checks | Local model; low compute |
| MPC [41] | Predicted state/input limits | Solver deadline; model sensitivity |

For motion planning, we adopt FASTER's principle of retaining a stopping backup in observed free space [22]. Short, low-speed local segments and an inventory-driven frontier fit the optical scan limit without the mixed-integer optimisation used by FASTER. This choice preserves a defined response to late map updates while matching the scan's optical speed limit.

## 3. System Design and Method

### 3.1. Hardware and installation

We select four T-MOTOR MN3508 KV380 motors, P14×4.8 carbon propellers and AIR40A 2–6S ESCs [6–8]. GEPRC specifies its 6S2P VTC6 pack at 6000 mAh, 607 g and 30 A continuous discharge [37]; we allocate 627 g installed. A plastic-case Pixhawk 6C and analog PM02 V3 provide flight control and voltage/current monitoring [10,11]. MID-360 supplies LiDAR/IMU observations, OAK-D Lite supplies forward stereo and navigation images, and an Orin Nano 8 GB module runs on a Hiwonder Mini carrier [12–15]. The 180 g computer allocation includes the 77 g carrier, module, cooling, storage and supports.

Each side head uses a BFS-U3-122S6M-C: 4096×3000 pixels, 3.45 μm pitch, global shutter, 36 g and 3 W maximum [35]. A 12 mm RT-V1228-MPY2 lens covers the 1.1-inch sensor [36]. Its 110 g installed allocation covers the 97 g catalogue lens, mounting and variation. Two pitch assemblies receive 70 g for servos, balance and supports [18]. Figure 2 links a reference mesh to these component roles; opposing cameras observe both racks during translation.

![Figure 2. Reference aircraft and selected components. Render: Skydio X2 [30]; product photos: T-MOTOR [6], Holybro [10], Opto Engineering [36]. Callouts identify proposed component roles.](figures/hardware.pdf)

We reject the 5 MP/8 mm option because its 55 g lenses reduce mass but require approximately 1.61 mm barcode modules under the baseline uncertainty envelope [16,17]. Our 12.3 MP installation adds 130 g in lenses/supports for improved sampling. With these optics, the 755 g 5 Ah LiPo exceeds the mass limit [9]. Flywoo's 596.2 g P30B pack saves 10.8 g, but its published 35 A description leaves continuous pack current unspecified [39]. We accept the GEPRC mass penalty for its documented continuous rating. Amprius's 465 g option supplies 12 A continuous, below the approximately 20 A low-voltage mission demand [44].

Table 0. Installed mass budget.

{{MASS_TABLE}}

The {{mass_g:.1f}} g total includes 100 g contingency and leaves {{mass_margin_g:.1f}} g to the limit. Custom structure uses two 180×140×2 mm carbon plates and four 16/14 mm outer/inner-diameter tubes, approximately 240 mm long. At 1600 kg/m³ these contribute 161 g and 72 g; the remainder of the 340 g frame allocation covers joints, fasteners and landing supports. Rings/bracing receive a separate 140 g. Substitutions must fit installed allocations, including cables and supports.

Adjacent motors are $0.600/\sqrt{2}=0.424$ m apart. With 0.3556 m rotors and two 0.020 m radial protection allowances, protected discs remain {{propeller_gap_m:.3f}} m apart. Rounding the {{geometry_outer_diameter_m:.3f}} m enclosing diameter gives a 1.00 m collision envelope of 0.50 m height. Side optical centres lie at $y=\pm0.40$ m, level with the body reference, with principal rays outside the rotors. Battery placement below the plates balances the elevated LiDAR and outboard optics.

Bank changes the swept envelope. For radius $r=0.50$ m and half-height $h=0.25$ m, lateral extent is $r\cos\theta+h\sin\theta$. At 12°, subtracting that extent and tracking/localisation/map allowances of 0.15/0.15/0.05 m from the 1.25 m half-aisle leaves {{side_clearance_budget_m:.3f}} m. At the 7.2 m upper scan height, vertical extent is $r\sin\theta+h\cos\theta$; allowances of 0.10/0.075/0.025 m leave {{ceiling_clearance_budget_m:.3f}} m ceiling clearance. Scan admission requires lateral sigma ≤0.05 m and vertical sigma ≤0.025 m; rack protrusions are checked against the same tilted envelope.

### 3.2. Power, interfaces and computation

A fused 6S branch feeds the ESCs; PM02 V3 powers Pixhawk separately. D36V50F12 regulates the computer, LiDAR and LEDs to 12 V; D36V50F6 supplies an isolated 6 V servo rail [11,33,34]. Regulation keeps the carrier below its 19 V maximum at 25.2 V full pack charge. Regulator selection uses a 60 W auxiliary allocation and thermal curves over 19.8–25.2 V.

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

At 200k points/s and 24 bytes/point, LiDAR adds 4.8 MB/s; 640×400 stereo at 30 Hz adds 15.36 MB/s, bringing raw input to about 266 MB/s. Paired exposures briefly produce 20 fps per side. Staggered USB transfer, bounded candidate queues and reduced-resolution detection followed by native crops limit the processing burst. The operating speed depends on sustaining <100 ms observation latency in the selected 15 W computer mode; overload selects slower motion or settled pairs. A 64 GB drive stores item records and selected crops.

### 3.3. Dynamics and flight control

We model position $\boldsymbol p$, velocity $\boldsymbol v$, body-to-world attitude $R$, body rate $\boldsymbol\omega$, total thrust $f$ and moment $\boldsymbol\tau$ in an upward-positive local frame [40]:

$$ \begin{aligned}\dot{\boldsymbol p}&=\boldsymbol v,\quad m\dot{\boldsymbol v}=Rf\boldsymbol e_3-mg\boldsymbol e_3+\boldsymbol d_f,\\ \dot R&=R[\boldsymbol\omega]_\times,\quad J\dot{\boldsymbol\omega}=\boldsymbol\tau-\boldsymbol\omega\times J\boldsymbol\omega+\boldsymbol d_\tau.\end{aligned} $$

The controller interface converts this convention to NED/FRD. Rotor thrust and reaction torque follow $f_i=k_T\Omega_i^2$ and $q_i=k_Q\Omega_i^2$, with first-order motor lag. Our reference simulation uses 40 ms lag and 200 Hz integration. The deployment cascade tracks position, velocity and acceleration, then computes orientation and allocates moments through PX4's attitude/rate loops [42].

For acceleration feedback we use:

$$ \boldsymbol a_c=\boldsymbol a_r+K_p(\boldsymbol p_r-\boldsymbol p)+K_d(\boldsymbol v_r-\boldsymbol v)+K_i\int(\boldsymbol p_r-\boldsymbol p)\,dt. $$

Choosing $K_p=\omega_n^2$, $K_d=2\zeta\omega_n$ makes bandwidth and damping explicit. Experimental horizontal gains $2.5~\mathrm{s}^{-2}$ and $2.6~\mathrm{s}^{-1}$ give 1.58 rad/s and 0.82; vertical gains 4.0 and 3.2 give 2.0 rad/s and 0.80. An attitude-response target of at least 2 Hz separates it from the approximately 0.25–0.32 Hz outer loop. Integration is clamped during saturation and frozen on invalid estimates. Loaded step responses determine deployment gains and filters.

Motor/ESC/propeller groups contribute approximately 0.026 kg m² roll/pitch inertia at 0.212 m axis offsets. Including structure, battery and optics gives a provisional 0.04–0.08 kg m² identification range. An illustrative 20% differential about hover produces 1.21 N m, subject to thrust and current. Current-aware allocation takes the form of weighted wrench fitting:

$$ \boldsymbol f^\star=\arg\min_{0\le f_i\le f_{max}(V)}\|W(B\boldsymbol f-\boldsymbol w_c)\|_2^2\quad\mathrm{s.t.}\quad\sum_i P_i(f_i,V)+P_{aux}\le VI_{max}. $$

Here $B$ maps thrusts to wrench, $\boldsymbol w_c$ is the demand, and $W$ prioritises roll/pitch and collective before yaw. Loaded power maps define $P_i$ and the shared continuous limit is $I_{max}=30$ A. This constraint defines deployment acceptance; the recorded reference controller uses inverse allocation with per-rotor clipping. At 0.85 m/s, 0.10 s latency and 0.6 m/s² braking imply 0.687 m stopping distance. A 1.5 m observed corridor includes mapping/tracking margins; a shorter corridor reduces admissible speed.

### 3.4. State estimation and local geometry

The navigation filter propagates $\boldsymbol x=(\boldsymbol p,\boldsymbol v,R,\boldsymbol b_a,\boldsymbol b_g)$ with MID-360 IMU measurements and corrects it with timestamped LiDAR/image residuals [21]. PX4 retains its own inertial stabilisation. IMU propagation extrapolates external odometry to control time; camera extrinsics, distortion, clock drift and exposure calibration share one transformation chain. At a linearised error-state update:

$$ \begin{aligned}P^-&=FP^+F^T+GQG^T,\quad S=HP^-H^T+R_z,\\ K&=P^-H^TS^{-1},\quad \delta\boldsymbol x=K\boldsymbol r.\end{aligned} $$

$R_z$ denotes measurement covariance, distinct from attitude $R$. FAST-LIVO2 iterates these updates [21,43]. Information conditioning, correspondence count, innovations and visual consistency are monitored jointly because repeated bays can produce a small residual at a wrong displacement. Weak along-aisle information triggers slower motion and a view of rack ends or cross-bracing. Exposure-aware image processing handles illumination changes while LiDAR supplies geometry. Map corrections enter a map-to-odometry transform, preserving a continuous control frame.

Our local voxel map distinguishes occupied, observed-free and unknown space, following OctoMap [25]. Planning uses a 0.10 m grid with underlying-point queries near obstacles; pruning preserves the return corridor and rack locations. Initialisation aligns gravity, defines an origin and fits floor/parallel rack planes. Upright clusters define bays; support and cargo surfaces define the task's five height bands. Observed surfaces define scan regions.

Initial geometry thresholds are RANSAC residual ≤max(0.03 m, 3σ_range), five consistent 10 Hz plane updates, <5° normal disagreement and 0.10 m upright/height clustering. These are initial tuning settings. Ambiguous bands pause association and request another view. Endpoint confirmation requires observed rack termination and a free inflated turn volume. Spatial association combines a 3σ gate, projected face overlap and side/band identity; ambiguous matches remain tentative. Empty slots require observed free faces.

MID-360's −7° to +52° elevation range leaves a lower blind region [12]. We mount forward stereo approximately 15° downward and yaw 180° at each endpoint so it faces the next pass. Four yaw/reacquisition actions cost 16 s. Keeping heading would save this time but require rear sensing or stricter LiDAR-only admission. Climb/descent requires recently observed free columns and ceiling clearance at the highest band. Figure 3 quantifies clearance and nominal ray limits; installed geometry supplies additional body/rotor occlusion checks.

![Figure 3. Motion limits and sensing envelope. Clearance includes tracking, localisation and map margins; the lower blind zone requires a previously mapped descent column.](figures/04_observability.pdf)

### 3.5. Coverage, confirmation and local recovery

At 5 Hz, the local planner receives state/covariance, the map, coverage records and remaining energy. It inflates obstacles by the tilted envelope, finds a short path and fits a timed segment with a stopping backup [22]. Collision, speed, acceleration and reserve checks precede replacement of the current plan. A late update leaves the accepted backup executable. A blocked endpoint selects another mapped transition column; otherwise we return with a blocked-region record.

Each cargo record uses side, bay/band and spatial track as its key, with payload, observation times, covariance and confirmation state as attributes. This preserves distinct instances of repeated SKUs. An action is admitted when:

$$ \mathcal A(a)=\mathcal L\land\mathcal F(a)\land\mathcal O(a)\land\left[E_{rem}\ge E_a+E_{unfinished}+E_{return}+0.20E_{del}\right]. $$

$\mathcal L$ checks current localisation; $\mathcal F$ checks the swept volume and backup; $\mathcal O$ checks image eligibility whenever a surface is marked inspected. Remaining workload $E_{unfinished}$ updates when cargo or blocked space is discovered. Transit alone cannot mark a surface complete.

Table 0. Inventory states and evidence required to advance.

| State | Input | Advance condition |
| --- | --- | --- |
| Initialise | IMU; initial geometry | Origin + free stopping corridor |
| Extend | Local map; rack bands | Admitted trajectory |
| Associate | Crops; poses; spatial gates | Update physical instance |
| Recover | Failed track; energy | Confirm or retain exception |
| Change level | Endpoint; transition column | Yaw and reacquire |
| Return | Coverage; tracks; reserve | Complete or report exceptions |

Each side carries two off-axis LUXEON Rebel LXML-PWC1-0100 LEDs in a 0.70 A series string driven by TPS92512 from 12 V [34]. A 30° beam covers about 0.16 m² at 0.85 m; 360 lm gives approximately 2 klx before losses, motivating a 0.20 ms exposure. The decoder detects candidates at reduced resolution, reads native crops with zxing-cpp [26], checks payload syntax and associates each crop using its exposure pose and calibrated pitch.

Confirmation requires consistent, quality-checked whole-label reads from two acquisitions of the same spatial track. Disagreement triggers a changed exposure or viewpoint. An unreadable carton remains an unresolved instance; confirmed aisle-visible units are counted per SKU and any encoded quantity is parsed separately. Empty, unseen, inaccessible and undecodable locations retain different states.

A failed read preserves opposite-side confirmations and freezes the unfinished boundary. Recovery starts with an in-view retry, followed by braking, recentering and an optional approach up to 0.10 m towards the failed side. Aimed pitch/height replaces the normal sweep during this settled read. With 0.70–0.85 m range and the adopted errors, the centred zero-pitch view retains a 0.454 m safe label-centre interval. A 0.10 m offset leaves 0.259 m lateral clearance against the 0.25 m target and improves centre-ray sampling by about 13%. Smaller free space or larger uncertainty reduces the offset. Both outward and return sweeps must pass collision checks before scanning resumes at the same boundary.

Each failed item receives at most two settled attempts. Completion requires both sides and all five bands inspected, every observed instance confirmed and no unchecked reachable surface boundary. Persistent failures produce an inventory with exceptions; reserve admission selects return when further recovery cannot be afforded.

## 4. Design Analysis

### 4.1. Complete-label visibility and acquisition

The selected optics give $f_x=12/0.00345=3478$ pixels and approximately 61.0° horizontal/46.7° vertical field. Our worked 250×40 mm Code128-B label carries 12 data symbols. Its 187 modules include start, checksum, stop and 10-module quiet zones on each side, giving $X=1.337$ mm. Other payload lengths change the module width; this operating case assumes warehouse agreement to larger labels.

Visibility is evaluated at all four label corners against the image frustum. For corner rays $q_k$, inward unit normals $n_j$, camera-orientation uncertainty $\delta$ and label-rotation displacement $\epsilon_l$, sufficient containment is:

$$ n_j^T q_k-\|q_k\|\sin\delta-\epsilon_l(1+\sin\delta)\ge0,\quad \epsilon_l=2r_l\sin(\alpha_l/2). $$

We enclose 0.80–0.95 m range, ±50 mm vertical error, ±10 mm along-track error, 2° orientation error, 5° label rotation and a 24-pixel border. Exposure admission uses measured rack-relative pose. The 2° optical cone describes settled acquisition; manoeuvres use the separate 12° collision envelope. Boundary pairs are acquired after acceleration settles. Vertex evaluation covers the continuous error box, with half a 1 mm height cell added to each interval.

The three poses in Figure 4 cover the 1.6 m band after complete-label erosion. Minimum usable along-track travel is 0.535 m. This treats a 40 mm-high label as a finite object, including at pose transitions. Each 0.20 s slot reserves 0.14 s for movement/settling and exposes at 0.14 and 0.19 s. Any two eligible observations at the same pitch may be paired, including adjacent cycles. Alternating event gaps of 0.05 and 0.55 s give a 0.60 s two-observation horizon; ±2 ms trigger jitter yields:

$$ v(T+2j)\le W_{safe},\qquad v\le0.535/0.604=0.885\ \mathrm{m/s}. $$

We select 0.85 m/s, leaving 21 mm of travel margin. Observations are separated by at least 46 ms and provide temporal corroboration; persistent glare or damaged bars motivates changed-view recovery. Minimum scale over admitted geometry is 2271 pixels/m: 250, 200 and 100 mm labels give 3.04, 2.43 and 1.21 pixels/module. Our 2.5 pixels/module target therefore excludes the latter two at this payload length. A 0.20 ms exposure limits translational blur to 0.74 pixel at 0.80 m. Loaded pitch settling, focus and contrast tests set final acquisition admission.

![Figure 4. Whole-label admission: (a) three-pitch coverage on the rack face; (b) corner containment schematic; (c) limiting travel budget.](figures/05_imaging.pdf)

### 4.2. Propulsion and shared-current authority

MN3508 KV380 manufacturer tests at 22.2 V give P14×4.8 samples of 710 g/64.38 W, 1100 g/117.66 W, 1350 g/166.5 W, 1580 g/217.56 W and 1730 g/255.3 W per motor [6]. Our 725.4 g-per-rotor hover lies within the first interval. With thrusts expressed in newtons, interpolation gives 265.94 W for four rotors:

$$ P_h=4\left[P_1+\frac{T_h-T_1}{T_2-T_1}(P_2-P_1)\right],\qquad T_h=mg/4. $$

Installation multipliers are 1.10 nominal and 1.25 adverse. Low-voltage screening scales maximum thrust by squared voltage and a 0.85 derating:

$$ \Gamma_{19.8}=\frac{4(1.730)(19.8/22.2)^2(0.85)}{2.9016}=1.61. $$

Simultaneous rotor maxima exceed the 30 A pack/harness limit. Collective and differential commands therefore follow the current-aware allocation in Section 3.3; the 1.61 ratio is a voltage-screening result, with the feasible collective/differential envelope determined by the loaded power map. Figure 5 shows measured propeller samples and installed-mass battery alternatives.

![Figure 5. Manufacturer power samples at 22.2 V [6] and installed-mass margin to 3 kg. The diamond marks hover interpolation.](figures/02_propulsion.pdf)

### 4.3. Mission duration and energy

Five bilateral passes require 400 m scan travel; return and launch approach add 84 m horizontally. Scan speed is 0.85 m/s, vertical speed is 0.35 m/s and acceleration is 0.6 m/s². Rest-to-rest travel over distance $d$ at speed limit $v$ takes:

$$ t(d)=\begin{cases}2\sqrt{d/a},&d\le v^2/a,\\ d/v+v/a,&d>v^2/a.\end{cases} $$

Initialisation allocates 5 s to stationary alignment, 3 s to local-plane acquisition and 2 s to health/admission checks. Each of four endpoints receives 0.5 s confirmation and 4 s yaw/reacquisition; ten boundary holds each consume one 0.60 s optical cycle. Discovery runs during the first scan. Unresolved geometry pauses the scan and consumes the remaining dwell allowance.

A failed item adds 7.66 s for processing, braking/resumption, recentering and two 0.5 s settled attempts. Optional lateral approach/return adds 2.61 s, including 0.30 s settling. Thus two, eight and twenty recovered items add 20.53, 82.14 and 205.32 s. Recovery counts are workload scenarios.

{{ENERGY_TABLE}}

The phase-power model separates installed hover loss, horizontal tilt/drag and positive climbing work. With installation factor $k$, drag area $C_D A=0.15~\mathrm{m}^2$ and phase duration $t_j$:

$$ E_j=\frac{\left[kP_h(1+(a/g)^2)^{3/4}+P_{aux}+\tfrac12\rho C_D A v^3\right]t_j+mg\max(0,\Delta z_j)/\eta_z}{3600}. $$

Vertical phases omit horizontal terms and use $\eta_z=0.60$; the adverse vertical multiplier is 1.10. Auxiliary load is 55/60 W nominal/adverse, with no descent energy credit. Applying the small tilt increment across a horizontal segment is our modelling approximation; loaded tests identify installation and rack-interaction losses.

VTC6 cell data specify 3.6 V nominal with rate-dependent discharge [38], while the pack page lists 22.2 V [37]. The 12-cell nameplate reference is 129.6 Wh; delivered energy is integrated down to 19.8 V under loaded conditions:

$$ E_{del}=\int_0^{t_*}V_{pack}(t)I(t)\,dt/3600,\qquad E_{mission}\le0.80E_{del}. $$

Two recoveries require {{required_delivered_wh:.2f}} Wh delivered before reserve; eight and twenty require {{required_eight:.2f}} and {{required_twenty:.2f}} Wh. The 100 Wh boundary in Figure 6 admits at most eight recoveries with little extra dwell, conditional on pack qualification. Twenty exceed that capacity. Lower temperature, longer discovery and larger installation loss reduce the admitted workload. Battery discharge qualification supplies the measured value of $E_{del}$ for flight release.

![Figure 6. Phase energy and the conditional 100 Wh boundary, including 20% reserve. Factors scale adverse hover power; markers denote 2, 8 and 20 recoveries.](figures/07_energy.pdf)

Table 0. Recovery workload under adverse power assumptions.

{{RETRY_TABLE}}

## 5. Experimental Evaluation

### 5.1. Implementation and evaluation conditions

We built a MuJoCo reference environment for flight tracking, continuous multilevel scanning and image-based inventory [27]. We use a Skydio X2 mesh from MuJoCo Menagerie, Poly Haven carton/shelf assets, and project geometry for primary rack supports and pallets [30–32]. The plant integrates free-body dynamics, four thrust sites, actuator lag, bounded control, contacts and bilateral rendered cameras. Table 9 identifies the configuration used for the recorded experiments alongside the proposed hardware.

Table 0. Design and experimental configurations.

| Parameter | Proposed aircraft | Reference experiment |
| --- | --- | --- |
| Installed mass | {{mass_g:.1f}} g allocation | 2.65 kg rigid body |
| Envelope | 1.00 m diameter; 0.50 m height | 0.80 m diameter; 0.29 m height |
| Battery | 6S2P 6 Ah; energy above 19.8 V | 4S 6 Ah model; 88.8 Wh |
| Propulsion | Matched table + installation factors | 330 W induced-power reference |
| Optics | 4096×3000; 12 mm; pitch sweep | 4096×2160; 45° horizontal; fixed |
| Scan speed | 0.85 m/s optical limit | 1.1 m/s prescribed route |
| Navigation | LiDAR/IMU/image estimator | Delayed/noisy position/velocity; ideal attitude |

The reference experiment uses a known route and state proxies to isolate tracking and rendered-image decoding. The online estimator, rack discovery and temporal-pair recovery constitute the next integration stage. Our 136 s technical video introduces the hardware and method, then replays the complete 501.02 s flight in 72 s at 6.96× speed. External, left/right and front-navigation views share the same saved pose and are re-rendered at 30 fps.

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

## 6. Discussion and Conclusion

Joint selection of optics, propulsion and task timing gives a {{mass_g:.1f}} g installation and a 0.85 m/s scanning envelope. Larger labels and higher-resolution cameras trade warehouse preparation and lens mass for continuous acquisition. Opposing heads avoid turning towards individual boxes; four endpoint yaws trade 16 s for forward navigation visibility. Immediate recovery retains completed work and limits extra travel, but its dwell still sets the feasible inventory workload.

Delivered battery energy sets the remaining feasibility condition. For the two-recovery case, {{adverse_energy_wh:.2f}} Wh mission energy requires {{required_delivered_wh:.2f}} Wh delivered before reserve. Hardware qualification comprises a loaded pack discharge at representative current and temperature, motor/pitch response measurements, and complete-label tests at the geometric limits. A subsequent unknown-layout flight would assess localisation, coverage, unresolved cargo and return reserve together. Tracking and decoding results provide initial evidence for that integration.

We provide [code and build instructions](../README.md), a [technical demonstration](https://youtu.be/ZQHlchVMqs8), [route metrics](../results/final/inference/metrics.json) and [inventory records](../results/final/inference/inventory.json). Calculations and figure inputs are reproducible; SHA-256 records identify the model and experiment files. Temporal min/max bands describe a single route. The code and video links are provided on the title page.

We used AI assistance for source discovery, implementation, calculation checking and drafting. Product images, algorithms and assets are attributed where used; the report has one author, Junjie Peng.

AWISU connects image eligibility, spatial coverage and energy through one action-admission rule. The resulting design supports a defined label and workload envelope, while the reference experiments demonstrate continuous serpentine tracking and image-based inventory. The next step is joint hardware and unknown-layout qualification under that same rule.
