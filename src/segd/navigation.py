"""Online LiDAR/IMU local mapping. No scene map or truth pose enters Navigator.

This compact error-state filter is a baseline, not FAST-LIO2 or stereo SLAM.
"""

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def skew(x):
    a, b, c = x
    return np.array([[0, -c, b], [c, 0, -a], [-b, a, 0]])


class SimulatedLidar:
    """Sensor simulator may access physical pose; only body-frame returns leave it."""

    def __init__(self, model, seed=0):
        self.model = model
        self.rng = np.random.default_rng(seed)
        az, el = np.meshgrid(
            np.linspace(-np.pi, np.pi, 120, endpoint=False),
            np.deg2rad([-7, 0, 15, 30, 50]),
        )
        self.directions = np.column_stack(
            (
                np.cos(el.ravel()) * np.cos(az.ravel()),
                np.cos(el.ravel()) * np.sin(az.ravel()),
                np.sin(el.ravel()),
            )
        )
        self.n = len(self.directions)
        self.ids = np.zeros(self.n, np.int32)
        self.ranges = np.zeros(self.n)

    def read(self, data):
        import mujoco

        R = data.xmat[1].reshape(3, 3)
        directions = np.ascontiguousarray((self.directions @ R.T).ravel())
        mujoco.mj_multiRay(
            self.model,
            data,
            data.xpos[1].copy(),
            directions,
            np.array([1, 0, 0, 1, 0, 0], np.uint8),
            True,
            1,
            self.ids,
            self.ranges,
            None,
            self.n,
            15.0,
        )
        self.downward_range = mujoco.mj_ray(
            self.model,
            data,
            data.xpos[1].copy(),
            -R[:, 2].copy(),
            np.array([1, 0, 0, 1, 0, 0], np.uint8),
            True,
            1,
            None,
        )
        valid = (self.ranges > 0.12) & (self.ranges < 15.0)
        distance = self.ranges[valid] + self.rng.normal(0, 0.008, valid.sum())
        return self.directions[valid] * distance[:, None]


class Navigator:
    def __init__(self, launch_position):
        self.p = np.array(launch_position, float)
        self.v = np.zeros(3)
        self.R = np.eye(3)
        self.ba = np.zeros(3)
        self.bg = np.zeros(3)
        self.P = np.diag(
            np.r_[
                np.full(3, 0.01**2),
                np.full(3, 0.03**2),
                np.full(3, 0.01**2),
                np.full(3, 0.02**2),
                np.full(3, 0.002**2),
            ]
        )
        self.points = np.empty((0, 3))
        self.normals = None
        self.tree = None
        self.last_key = self.p.copy()
        self.quality = {"valid": False, "inliers": 0, "rmse": None, "condition": None}

    def propagate(self, accelerometer, gyro, dt):
        a = np.asarray(accelerometer) - self.ba
        w = np.asarray(gyro) - self.bg
        world_a = self.R @ a + np.array([0, 0, -9.81])
        self.p += self.v * dt + 0.5 * world_a * dt * dt
        self.v += world_a * dt
        F = np.eye(15)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -self.R @ skew(a) * dt
        F[3:6, 9:12] = -self.R * dt
        F[6:9, 12:15] = -np.eye(3) * dt
        self.R = self.R @ Rotation.from_rotvec(w * dt).as_matrix()
        Q = (
            np.diag(
                np.r_[
                    np.full(3, 1e-7),
                    np.full(3, 0.025**2),
                    np.full(3, 0.002**2),
                    np.full(3, 1e-7),
                    np.full(3, 1e-9),
                ]
            )
            * dt
        )
        self.P = F @ self.P @ F.T + Q

    def update_height(self, downward_range):
        # Warehouse floor is assumed level, with launch-pad floor defining z=0.
        height = downward_range * self.R[2, 2]
        if not 0 < height < 9 or self.R[2, 2] < 0.94:
            return
        H = np.zeros((1, 15))
        H[0, 2] = 1
        noise = 0.01**2
        K = self.P @ H.T / (float((H @ self.P @ H.T)[0, 0]) + noise)
        delta = K[:, 0] * (height - self.p[2])
        self.p += delta[:3]
        self.v += delta[3:6]
        self.ba += delta[9:12]
        self.R = self.R @ Rotation.from_rotvec(delta[6:9]).as_matrix()
        self.bg += delta[12:15]
        I = np.eye(15) - K @ H
        self.P = I @ self.P @ I.T + (K @ K.T) * noise

    def _map(self, cloud):
        points = np.vstack((self.points, cloud))
        _, indices = np.unique(
            np.round(points / 0.10).astype(int), axis=0, return_index=True
        )
        points = points[np.sort(indices)]
        local = np.linalg.norm(points - self.p, axis=1) < 18
        self.points = points[local][-16000:]
        self.tree = cKDTree(self.points)
        _, indices = self.tree.query(self.points, k=min(10, len(self.points)))
        neighbours = self.points[indices]
        centred = neighbours - neighbours.mean(axis=1, keepdims=True)
        covariance = np.einsum("nki,nkj->nij", centred, centred)
        values, vectors = np.linalg.eigh(covariance)
        self.normals = vectors[:, :, 0]
        self.planar = values[:, 0] / np.maximum(values.sum(axis=1), 1e-10) < 0.03
        self.last_key = self.p.copy()

    def update(self, body_points):
        if len(body_points) < 40:
            self.quality = {
                "valid": False,
                "inliers": len(body_points),
                "rmse": None,
                "condition": None,
            }
            return False
        if self.tree is None:
            self._map(body_points @ self.R.T + self.p)
            self.quality = {
                "valid": True,
                "inliers": len(body_points),
                "rmse": 0.0,
                "condition": None,
            }
            return True
        p = self.p.copy()
        R = self.R.copy()
        good = None
        for _ in range(8):
            rotated = body_points @ R.T
            cloud = rotated + p
            distances, idx = self.tree.query(cloud)
            good = distances < 0.35
            if good.sum() < 40:
                break
            normals = self.normals[idx[good]]
            residual = np.sum(normals * (cloud[good] - self.points[idx[good]]), axis=1)
            H = np.column_stack((normals, np.cross(rotated[good], normals)))
            weights = np.minimum(1, 0.05 / np.maximum(abs(residual), 1e-8))
            info = H.T @ (weights[:, None] * H)
            delta = np.linalg.solve(
                info + np.eye(6) * 0.02, -H.T @ (weights * residual)
            )
            p += delta[:3]
            R = Rotation.from_rotvec(delta[3:]).as_matrix() @ R
            if np.linalg.norm(delta) < 1e-4:
                break
        valid = good is not None and good.sum() >= 40
        if valid:
            rmse = float(np.sqrt(np.mean(residual**2)))
            condition = float(np.linalg.cond(info + np.eye(6) * 1e-6))
            valid = (
                rmse < 0.10
                and np.linalg.norm(p - self.p) < 0.6
                and np.linalg.norm(Rotation.from_matrix(self.R.T @ R).as_rotvec())
                < 0.10
                and condition < 1e7
            )
        else:
            rmse = condition = None
        self.quality = {
            "valid": bool(valid),
            "inliers": int(good.sum()) if good is not None else 0,
            "rmse": rmse,
            "condition": condition,
        }
        if not valid:
            return False
        innovation = np.r_[p - self.p, Rotation.from_matrix(self.R.T @ R).as_rotvec()]
        Hf = np.zeros((6, 15))
        Hf[:3, :3] = np.eye(3)
        Hf[3:, 6:9] = np.eye(3)
        N = np.diag([0.025**2, 0.025**2, 0.15**2] + [0.012**2] * 3)
        K = np.linalg.solve(Hf @ self.P @ Hf.T + N, Hf @ self.P).T
        correction = K @ innovation
        self.p += correction[:3]
        self.v += correction[3:6]
        self.R = self.R @ Rotation.from_rotvec(correction[6:9]).as_matrix()
        self.ba += correction[9:12]
        self.bg += correction[12:15]
        I = np.eye(15) - K @ Hf
        self.P = I @ self.P @ I.T + K @ N @ K.T
        if np.linalg.norm(self.p - self.last_key) > 0.35:
            self._map(body_points @ self.R.T + self.p)
        return True


class StereoOdometry:
    """Sparse stereo triangulation and inter-frame PnP, from rendered pixels only."""

    def __init__(self, model):
        import cv2, mujoco

        self.model = model
        self.renderer = mujoco.Renderer(model, height=540, width=960)
        self.orb = cv2.ORB_create(nfeatures=1800, fastThreshold=10)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        f = 540 / (2 * np.tan(np.deg2rad(50) / 2))
        self.K = np.array([[f, 0, 480], [0, f, 270], [0, 0, 1.0]])
        self.Rbc = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0.0]])
        self.tbc = np.array([0.10, 0.04, 0.04])
        self.previous = None
        self.valid_updates = 0
        self.rejected_updates = 0

    def update(self, data, navigator):
        import cv2

        images = []
        for camera in ["front_left", "front_right"]:
            from .rendering import sensor_frame

            images.append(
                cv2.cvtColor(
                    sensor_frame(self.renderer, self.model, data, camera),
                    cv2.COLOR_RGB2GRAY,
                )
            )
        accepted = False
        left, dl = self.orb.detectAndCompute(images[0], None)
        right, dr = self.orb.detectAndCompute(images[1], None)
        if dl is None or dr is None:
            return False
        if self.previous is not None:
            prev = self.previous
            matches = self.matcher.knnMatch(prev["descriptors"], dl, k=2)
            good = [
                a
                for pair in matches
                if len(pair) == 2
                for a, b in [pair]
                if a.distance < 0.72 * b.distance and a.queryIdx in prev["points"]
            ]
            if len(good) >= 20:
                xyz = np.array([prev["points"][m.queryIdx] for m in good], np.float32)
                uv = np.array([left[m.trainIdx].pt for m in good], np.float32)
                ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                    xyz,
                    uv,
                    self.K,
                    None,
                    iterationsCount=80,
                    reprojectionError=2.5,
                    confidence=0.995,
                )
                if ok and inliers is not None and len(inliers) >= 18:
                    relR = cv2.Rodrigues(rvec)[0]
                    relt = tvec.ravel()
                    Rwc = prev["R"] @ self.Rbc
                    pwc = prev["p"] + prev["R"] @ self.tbc
                    current_Rwc = Rwc @ relR.T
                    current_pwc = pwc - current_Rwc @ relt
                    measured_R = current_Rwc @ self.Rbc.T
                    measured_p = current_pwc - measured_R @ self.tbc
                    if (
                        np.linalg.norm(measured_p - navigator.p) < 0.5
                        and np.linalg.norm(
                            Rotation.from_matrix(navigator.R.T @ measured_R).as_rotvec()
                        )
                        < 0.15
                    ):
                        dt = max(1e-3, float(data.time) - prev["t"])
                        measured_v = (measured_p - prev["p"]) / dt
                        H = np.zeros((6, 15))
                        H[:3, :3] = np.eye(3)
                        H[3:, 3:6] = np.eye(3)
                        N = np.diag([0.05**2] * 3 + [0.04**2] * 3)
                        K = np.linalg.solve(
                            H @ navigator.P @ H.T + N, H @ navigator.P
                        ).T
                        delta = (
                            K
                            @ np.r_[measured_p - navigator.p, measured_v - navigator.v]
                        )
                        navigator.p += delta[:3]
                        navigator.v += delta[3:6]
                        navigator.ba += delta[9:12]
                        navigator.R = (
                            navigator.R @ Rotation.from_rotvec(delta[6:9]).as_matrix()
                        )
                        navigator.bg += delta[12:15]
                        I = np.eye(15) - K @ H
                        navigator.P = I @ navigator.P @ I.T + K @ N @ K.T
                        self.valid_updates += 1
                        accepted = True
                    else:
                        self.rejected_updates += 1
        # Build metric points for the next temporal match from left/right observations.
        matches = self.matcher.knnMatch(dl, dr, k=2)
        points = {}
        f = self.K[0, 0]
        for pair in matches:
            if len(pair) != 2:
                continue
            a, b = pair
            if a.distance >= 0.7 * b.distance:
                continue
            ul, vl = left[a.queryIdx].pt
            ur, vr = right[a.trainIdx].pt
            disparity = ul - ur
            if abs(vl - vr) > 1.5 or not 2 < disparity < 150:
                continue
            depth = f * 0.08 / disparity
            points[a.queryIdx] = [(ul - 480) * depth / f, (vl - 270) * depth / f, depth]
        self.previous = {
            "descriptors": dl,
            "points": points,
            "p": navigator.p.copy(),
            "R": navigator.R.copy(),
            "t": float(data.time),
        }
        return accepted

    def close(self):
        self.renderer.close()
