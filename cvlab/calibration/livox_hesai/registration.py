"""Target-free Livox <-> Hesai registration: robust point-to-plane ICP of the Hesai points (source)
onto the Livox surfaces (target), coarse to fine, over one or several static stops at once.

Every stop is a different scene seen from the same rig, so the stops share one transform but never
share correspondences: the normal equations of all the stops are summed into one 6x6 system.
Frames: camera convention of cvlab.lidar (x right, y down, z forward), metres here.
Transform: P_livox = R P_hesai + t, as a 4x4 matrix.
"""

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

MIN_RANGE_M, MAX_RANGE_M = 0.5, 30.0
# (voxel m, max correspondence distance m): from a +-10 deg / +-30 cm guess down to the 2 cm level
SCHEDULE = [(0.20, 1.0), (0.10, 0.5), (0.05, 0.2), (0.02, 0.08)]
MAX_ITERATIONS = 30
MIN_NEIGHBOURS = 6           # points within the normal radius for a target normal to count
PARAMETERS = ["rot x", "rot y", "rot z", "t x (right)", "t y (down)", "t z (forward)"]


def in_range(xyz_m):
    r = np.linalg.norm(xyz_m, axis=1)
    return xyz_m[(r > MIN_RANGE_M) & (r < MAX_RANGE_M)]


def downsample(xyz_m, voxel):
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz_m.astype(np.float64)))
    return np.asarray(pc.voxel_down_sample(voxel).points)


class Target:
    """Livox points of a stop at one voxel size, with normals and a KD-tree."""

    def __init__(self, xyz_m, voxel):
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(downsample(xyz_m, voxel)))
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 3, max_nn=30))
        self.points = np.asarray(pc.points)
        self.normals = np.asarray(pc.normals)
        self.tree = cKDTree(self.points)
        # A normal is only meaningful on a surface sampled densely enough around the point
        counts = self.tree.query_ball_point(self.points, voxel * 3, return_length=True)
        self.good = counts >= MIN_NEIGHBOURS


def residuals(source, target, T, max_distance):
    """Point-to-plane residuals (m), Jacobian (N, 6) and correspondence mask of source under T."""
    p = source @ T[:3, :3].T + T[:3, 3]
    d, idx = target.tree.query(p, distance_upper_bound=max_distance)
    found = np.isfinite(d)
    found[found] = target.good[idx[found]]
    p, q, n = p[found], target.points[idx[found]], target.normals[idx[found]]
    r = np.einsum("ij,ij->i", p - q, n)
    # Left update T <- exp([w, v]) T: p' = p + w x p + v, so dr = (p x n) . w + n . v
    J = np.hstack([np.cross(p, n), n])
    return r, J, found


def align(stops, T0, schedule=SCHEDULE, log=lambda *a: None):
    """stops: [(hesai_xyz_m, livox_xyz_m)]. Returns (T, report) with report = {"stops": per stop
    {"fitness", "rmse_m", "pairs"}, "hessian": 6x6 of the final level, "rmse_m"}."""
    T = np.array(T0, dtype=np.float64)
    for voxel, max_distance in schedule:
        sources = [downsample(h, voxel) for h, _ in stops]
        targets = [Target(lv, voxel) for _, lv in stops]
        scale = max_distance / 3    # Cauchy weights: a residual of `scale` gets half the weight
        for _ in range(MAX_ITERATIONS):
            H, g = np.zeros((6, 6)), np.zeros(6)
            for source, target in zip(sources, targets):
                r, J, _ = residuals(source, target, T, max_distance)
                w = 1.0 / (1.0 + (r / scale) ** 2)
                H += J.T @ (J * w[:, None])
                g += J.T @ (w * r)
            if not np.any(H):
                raise ValueError("no overlap between the two lidars: check the initial guess")
            delta = -np.linalg.solve(H + 1e-9 * np.eye(6), g)
            update = np.eye(4)
            update[:3, :3] = Rotation.from_rotvec(delta[:3]).as_matrix()
            update[:3, 3] = delta[3:]
            T = update @ T
            if np.linalg.norm(delta[:3]) < 1e-6 and np.linalg.norm(delta[3:]) < 1e-5:
                break
        log(f"\tvoxel {voxel * 100:.0f} cm: {describe(T)}")

    report = {"stops": []}
    H_total, sq, n_total = np.zeros((6, 6)), 0.0, 0
    for source, target in zip(sources, targets):
        r, J, found = residuals(source, target, T, max_distance)
        inliers = np.abs(r) < scale
        H_total += J[inliers].T @ J[inliers]
        sq += np.sum(r[inliers] ** 2)
        n_total += inliers.sum()
        report["stops"].append({"fitness": float(inliers.sum() / len(source)),
                                "rmse_m": float(np.sqrt(np.mean(r[inliers] ** 2))) if inliers.any() else None,
                                "pairs": int(inliers.sum())})
    report["hessian"] = H_total
    report["rmse_m"] = float(np.sqrt(sq / max(n_total, 1)))
    return T, report


def conditioning(H, lever_m=5.0):
    """How well the surfaces constrain each motion. Rotations are compared with translations through
    their effect at lever_m (a rotation of 1 / lever_m rad moves a point at lever_m by 1 m).
    Returns [{"direction", "strength"}] weakest first, strength relative to the strongest."""
    S = np.diag([lever_m] * 3 + [1.0] * 3)
    Hs = np.linalg.inv(S) @ H @ np.linalg.inv(S)
    values, vectors = np.linalg.eigh(Hs)
    out = []
    for value, vector in zip(values, vectors.T):
        weights = vector ** 2
        main = np.argsort(-weights)[:2]
        name = PARAMETERS[main[0]] + (f" + {PARAMETERS[main[1]]}" if weights[main[1]] > 0.25 else "")
        out.append({"direction": name, "strength": float(np.sqrt(max(value, 0) / values.max()))})
    return out


def describe(T):
    rot = Rotation.from_matrix(T[:3, :3])
    return (f"t = {np.round(T[:3, 3] * 1000, 1)} mm, rot = {np.round(np.degrees(rot.as_rotvec()), 2)} deg")


def difference(T1, T2):
    """(translation mm, rotation deg) between two transforms."""
    dt = np.linalg.norm(T1[:3, 3] - T2[:3, 3]) * 1000
    dr = np.degrees(Rotation.from_matrix(T1[:3, :3] @ T2[:3, :3].T).magnitude())
    return float(dt), float(dr)
