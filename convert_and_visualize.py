"""
Convert camera poses from Robot/OpenCV RH world to Unreal Engine LH world,
then visualize in a simulated Unreal Engine coordinate frame.

Input:  e1.json .. e4.json  — 3x4 [R|t] camera-to-world in RH OpenCV convention
        OpenCV camera axes: X=Right, Y=Down, Z=Forward (optical axis)
        World: right-handed

Output: e1_ue.json .. e4_ue.json — 4x4 camera-to-world in UE LH convention
        UE world axes: X=Forward, Y=Right, Z=Up (left-handed)
        Units: centimeters

Derivation (see derive_from_scratch.md):
  Step 1: Axis remap  — OpenCV camera (X-right,Y-down,Z-fwd) → UE actor (X-fwd,Y-right,Z-up)
          R_ue2cv columns = UE axes in OpenCV basis:
            col0=[0,0,1], col1=[1,0,0], col2=[0,-1,0]
          R_remapped = R_WC @ R_ue2cv       (still RH)
          t_remapped = t_WC                 (unchanged, still in RH world)

  Step 2: Handedness flip  — RH world → LH world (negate Y)
          S = diag(1, -1, 1)
          R_LH = S @ R_remapped @ S
          t_LH = S @ t_remapped
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# ---------------------------------------------------------------------------
# Conversion
# ---------------------------------------------------------------------------

def load_camera_pose(json_path):
    """Load a 3x4 [R|t] camera-to-world matrix from JSON (RH OpenCV)."""
    with open(json_path) as f:
        data = json.load(f)
    mat = np.array(data["matrix"])          # (3, 4)
    R = mat[:, :3]
    t = mat[:, 3]
    return R, t


def opencv_rh_to_ue_lh(R_wc, t_wc):
    """
    Convert a camera-to-world pose from OpenCV/RH to Unreal Engine/LH.

    Parameters
    ----------
    R_wc : (3,3) rotation matrix, camera-to-world, right-handed
    t_wc : (3,)  translation vector (meters), camera position in RH world

    Returns
    -------
    R_lh : (3,3) rotation matrix, camera-to-world, left-handed (det ≈ -1)
    t_lh : (3,)  translation vector (meters), camera position in LH world
    """
    # Step 1 – axis remap (OpenCV cam → UE actor frame, still RH)
    # Columns = each UE axis expressed in the OpenCV basis
    #   col 0: UE X (forward)  = OpenCV Z   → [0, 0, 1]
    #   col 1: UE Y (right)    = OpenCV X   → [1, 0, 0]
    #   col 2: UE Z (up)       = OpenCV -Y  → [0,-1, 0]
    R_ue2cv = np.array([
        [0,  1,  0],
        [0,  0, -1],
        [1,  0,  0],
    ], dtype=float)

    R_remapped = R_wc @ R_ue2cv            # world-from-UE-camera, still RH
    t_remapped = t_wc.copy()               # position unchanged in RH world

    # Step 2 – handedness flip (negate Y to go RH → LH)
    S = np.diag([1.0, -1.0, 1.0])
    R_lh = S @ R_remapped @ S
    t_lh = S @ t_remapped

    return R_lh, t_lh


def rotation_to_ue_euler(R_lh):
    """
    Extract UE-convention Euler angles (Roll, Pitch, Yaw) in degrees
    from a left-handed camera-to-world rotation matrix.

    The columns of R_lh are the actor's local axes in UE world space:
      col 0 = forward (X),  col 1 = right (Y),  col 2 = up (Z)

    UE convention:
      Yaw   = rotation around Z (up)    — positive CW from above
      Pitch = rotation around Y (right)  — positive nose-up
      Roll  = rotation around X (forward) — positive CW looking forward

    We extract directly from the forward vector and the matrix elements.
    """
    fwd = R_lh[:, 0]   # actor forward in world

    # Yaw: angle of forward projected onto XY plane, from +X toward +Y
    yaw = np.degrees(np.arctan2(fwd[1], fwd[0]))

    # Pitch: elevation of forward above XY plane
    pitch = np.degrees(np.arctan2(-fwd[2],
                                   np.sqrt(fwd[0]**2 + fwd[1]**2)))

    # Roll: once yaw+pitch are removed, remaining twist around forward
    # Build yaw-pitch rotation, invert, apply to R_lh, read off roll
    cy, sy = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    cp, sp = np.cos(np.radians(pitch)), np.sin(np.radians(pitch))

    # Rz(yaw) — LH convention (positive = CW from above → negate sin)
    Rz = np.array([[ cy, sy, 0],
                    [-sy, cy, 0],
                    [  0,  0, 1]], dtype=float)
    # Ry(pitch) — standard
    Ry = np.array([[ cp, 0, sp],
                    [  0, 1,  0],
                    [-sp, 0, cp]], dtype=float)

    R_yp = Rz @ Ry
    R_roll = R_yp.T @ R_lh   # residual should be Rx(roll)
    roll = np.degrees(np.arctan2(R_roll[1, 2], R_roll[2, 2]))

    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------

def export_ue_json(cam_name, R_lh, t_lh_m, out_path):
    """Write a single camera's UE pose to JSON (position in cm)."""
    t_cm = t_lh_m * 100.0
    roll, pitch, yaw = rotation_to_ue_euler(R_lh)
    det = float(np.linalg.det(R_lh))

    T = np.eye(4)
    T[:3, :3] = R_lh
    T[:3, 3]  = t_lh_m

    data = {
        "camera_name": cam_name,
        "coordinate_system": "LEFT-HANDED (Unreal Engine)",
        "axes_convention": "X=Forward, Y=Right, Z=Up",
        "position_cm": {"x": round(t_cm[0], 4),
                        "y": round(t_cm[1], 4),
                        "z": round(t_cm[2], 4)},
        "position_m":  {"x": round(t_lh_m[0], 6),
                        "y": round(t_lh_m[1], 6),
                        "z": round(t_lh_m[2], 6)},
        "rotation_deg": {"roll": round(roll, 4),
                         "pitch": round(pitch, 4),
                         "yaw": round(yaw, 4)},
        "rotation_matrix": [[round(v, 6) for v in row] for row in R_lh.tolist()],
        "rotation_matrix_det": round(det, 6),
        "transform_4x4": [[round(v, 6) for v in row] for row in T.tolist()],
    }
    with open(out_path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"  wrote {out_path}")


# ---------------------------------------------------------------------------
# Interactive 3D visualisation helpers
# ---------------------------------------------------------------------------

def _draw_frame_3d(ax, R, t, label, length=0.025, lw=2, to_plot=None):
    """Draw a coordinate frame (RGB = XYZ) on a 3D axes.

    to_plot: optional callable  v_ue → v_mpl  for coordinate remapping.
             If None, identity (no remap).
    """
    if to_plot is None:
        to_plot = lambda v: v

    origin_p = to_plot(t)
    ax.scatter(*origin_p, s=60, c='k', zorder=5)
    ax.text(origin_p[0] + length * 0.3, origin_p[1] + length * 0.3, origin_p[2],
            label, fontsize=8, ha='left', fontweight='bold')

    for i, c in enumerate(['r', 'g', 'b']):
        tip = t + R[:, i] * length
        tip_p = to_plot(tip)
        ax.plot([origin_p[0], tip_p[0]],
                [origin_p[1], tip_p[1]],
                [origin_p[2], tip_p[2]],
                color=c, linewidth=lw)


# ---------------------------------------------------------------------------
# Simulated Unreal Engine visualiser  (interactive, LEFT-HANDED)
# ---------------------------------------------------------------------------

def visualize_both(cameras_rh, cameras_lh):
    """
    Single interactive window with RH (left panel) and UE LH (right panel)
    side by side.  Both subplots are 3D and can be rotated independently.
    """
    import matplotlib.ticker as mticker

    fig = plt.figure(figsize=(20, 9))

    # =====================  LEFT: Original RH  =====================
    ax_rh = fig.add_subplot(121, projection='3d')

    o = np.zeros(3)
    ax_rh.scatter(*o, s=200, c='black', marker='o', zorder=10)

    axis_len = 0.08
    for i, (c, name) in enumerate(zip(['r', 'g', 'b'], ['X', 'Y', 'Z'])):
        tip = np.zeros(3); tip[i] = axis_len
        ax_rh.plot([0, tip[0]], [0, tip[1]], [0, tip[2]],
                   color=c, linewidth=4)
        ax_rh.text(tip[0], tip[1], tip[2], f'  {name}',
                   fontsize=10, fontweight='bold', color=c)

    for cam_name, (R, t) in cameras_rh.items():
        _draw_frame_3d(ax_rh, R, t, cam_name.upper(), length=0.025, lw=2)

    ax_rh.set_xlabel('X (m)', fontsize=10)
    ax_rh.set_ylabel('Y (m)', fontsize=10)
    ax_rh.set_zlabel('Z (m)', fontsize=10)
    ax_rh.set_title('Original Robot/OpenCV\n(Right-Handed)',
                    fontsize=12, fontweight='bold', pad=10)
    lim = 0.15
    ax_rh.set_xlim(-lim, lim); ax_rh.set_ylim(-lim, lim)
    ax_rh.set_zlim(-lim, lim)
    ax_rh.view_init(elev=25, azim=-60)
    ax_rh.grid(True, alpha=0.3)

    # =====================  RIGHT: UE LH  =====================
    ax_ue = fig.add_subplot(122, projection='3d')

    # Negate Y in data to convert LH → RH for matplotlib rendering
    def to_plot(v):
        return np.array([v[0], -v[1], v[2]])

    o_p = to_plot(np.zeros(3))
    ax_ue.scatter(*o_p, s=200, c='black', marker='o', zorder=10)

    for i, (c, name) in enumerate(zip(['r', 'g', 'b'],
                                       ['X (Fwd)', 'Y (Right)', 'Z (Up)'])):
        tip_ue = np.zeros(3); tip_ue[i] = axis_len
        tip_p = to_plot(tip_ue)
        ax_ue.plot([o_p[0], tip_p[0]], [o_p[1], tip_p[1]], [o_p[2], tip_p[2]],
                   color=c, linewidth=4)
        ax_ue.text(tip_p[0], tip_p[1], tip_p[2], f'  {name}',
                   fontsize=10, fontweight='bold', color=c)

    for cam_name, (R_lh, t_lh) in cameras_lh.items():
        _draw_frame_3d(ax_ue, R_lh, t_lh, cam_name.upper(),
                       length=0.025, lw=2, to_plot=to_plot)

    # Fix Y tick labels (data is negated, show true UE values)
    ax_ue.yaxis.set_major_formatter(
        mticker.FuncFormatter(lambda v, _: f'{-v:.2f}'))

    ax_ue.set_xlabel('UE X (Forward) →', fontsize=10, labelpad=8)
    ax_ue.set_ylabel('UE Y (Right) →', fontsize=10, labelpad=8)
    ax_ue.set_zlabel('UE Z (Up) ↑', fontsize=10, labelpad=8)
    ax_ue.set_title('Simulated Unreal Engine\n(Left-Handed: X=Fwd, Y=Right, Z=Up)',
                    fontsize=12, fontweight='bold', pad=10)
    ax_ue.set_xlim(-lim, lim); ax_ue.set_ylim(-lim, lim)
    ax_ue.set_zlim(-lim / 2, lim)
    ax_ue.view_init(elev=25, azim=-60)
    ax_ue.grid(True, alpha=0.3)

    plt.tight_layout()
    out_png = 'cameras_side_by_side.png'
    plt.savefig(out_png, dpi=150, bbox_inches='tight')
    print(f"\n  Saved → {out_png}")
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    cam_files = ['e1.json', 'e2.json', 'e3.json', 'e4.json']

    cameras_rh = {}   # name → (R, t) in RH
    cameras_lh = {}   # name → (R, t) in LH

    print("=" * 65)
    print(" OpenCV/RH  →  Unreal Engine/LH  camera pose conversion")
    print("=" * 65)

    for cf in cam_files:
        name = cf.replace('.json', '')
        R_rh, t_rh = load_camera_pose(cf)
        cameras_rh[name] = (R_rh, t_rh)

        det_rh = np.linalg.det(R_rh)
        R_lh, t_lh = opencv_rh_to_ue_lh(R_rh, t_rh)
        det_lh = np.linalg.det(R_lh)

        cameras_lh[name] = (R_lh, t_lh)

        roll, pitch, yaw = rotation_to_ue_euler(R_lh)

        print(f"\n{name.upper()}")
        print(f"  RH  t = [{t_rh[0]:.4f}, {t_rh[1]:.4f}, {t_rh[2]:.4f}] m   det(R)={det_rh:+.4f}")
        print(f"  LH  t = [{t_lh[0]:.4f}, {t_lh[1]:.4f}, {t_lh[2]:.4f}] m   det(R)={det_lh:+.4f}")
        print(f"  UE Euler  roll={roll:.2f}°  pitch={pitch:.2f}°  yaw={yaw:.2f}°")

        export_ue_json(name, R_lh, t_lh, f"{name}_ue.json")

    # Verify: inter-camera distances should be identical in both frames
    print("\n" + "-" * 65)
    print(" Sanity check: inter-camera distances (meters)")
    print("-" * 65)
    names = list(cameras_rh.keys())
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            d_rh = np.linalg.norm(cameras_rh[names[i]][1] - cameras_rh[names[j]][1])
            d_lh = np.linalg.norm(cameras_lh[names[i]][1] - cameras_lh[names[j]][1])
            ok = "✓" if abs(d_rh - d_lh) < 1e-10 else "✗"
            print(f"  {names[i]}↔{names[j]}  RH={d_rh:.6f}  LH={d_lh:.6f}  {ok}")

    # Visualise
    print("\n" + "=" * 65)
    print(" Generating visualisations …")
    print("=" * 65)
    visualize_both(cameras_rh, cameras_lh)

    print("\n  Close the window to exit.")
    plt.show()


if __name__ == "__main__":
    main()
