# Converting Camera Pose from Robotics (OpenCV) to Unreal Engine

## Goal

In many robotics and computer vision systems, camera poses are defined using a **right-handed** coordinate convention where the camera optical axis is **Z-forward**, **X-right**, and **Y-down** (the OpenCV/ROS convention).

Unreal Engine uses a **left-handed** coordinate system where the camera frame aligns with the world frame: **X-forward**, **Y-right**, and **Z-up**.

The goal is to convert a 4×4 camera pose matrix expressed in the robotics convention into an equivalent transform that can be consumed by Unreal Engine.

## Quick Start

```bash
python3 convert_and_visualize.py
```

This reads `e1.json`–`e4.json`, writes `e1_ue.json`–`e4_ue.json`, and opens an interactive side-by-side 3D visualization (RH vs UE LH).

| Axis | OpenCV camera frame | UE camera/world frame  |
|------|---------------------|------------------------|
| X    | Right               | **Forward**            |
| Y    | Down                | **Right**              |
| Z    | Forward (optical)   | **Up**                 |

**Key difference:** In OpenCV, the camera uses a special optical axis convention. In Unreal Engine, the camera frame matches the world frame convention.

## The Math

### Step 0: Input (Robotics Convention)

A camera pose in robotics is a 4×4 homogeneous **camera-to-world** transform:

$$
\mathbf{T}_{WC}^{\text{RH}} = \begin{bmatrix}
\mathbf{R}_{WC}^{\text{RH}} & \mathbf{t}_{WC} \\
\mathbf{0}^T & 1
\end{bmatrix}
$$

where $\mathbf{R}_{WC}^{\text{RH}} \in \mathbb{R}^{3 \times 3}$ is a rotation matrix ($\det(\mathbf{R}_{WC}^{\text{RH}}) = +1$, right-handed) and $\mathbf{t}_{WC} \in \mathbb{R}^3$ is the camera position in world coordinates.

The columns of $\mathbf{R}_{WC}^{\text{RH}}$ are the camera's local axes (OpenCV convention) expressed in world coordinates:

- Column 0 = camera **X** (right) direction in world
- Column 1 = camera **Y** (down) direction in world
- Column 2 = camera **Z** (forward/optical) direction in world

### Step 1 — Axis Remapping (OpenCV → UE, still RH)

Convert the camera axes from the OpenCV convention $(x_{cv}, y_{cv}, z_{cv}) = (\text{right}, \text{down}, \text{forward})$ into Unreal's camera convention $(x_{ue}, y_{ue}, z_{ue}) = (\text{forward}, \text{right}, \text{up})$.

Define $\mathbf{R}_{ue \rightarrow cv}$: a matrix whose columns are the **UE axes expressed in the OpenCV basis** (i.e. it maps UE camera coordinates to OpenCV camera coordinates):

$$
\mathbf{R}_{ue \rightarrow cv} = \begin{bmatrix}
0 & 1 & 0 \\
0 & 0 & -1 \\
1 & 0 & 0
\end{bmatrix}
\qquad \det(\mathbf{R}_{ue \rightarrow cv}) = -1
$$

This reads as:

- New **X** (forward) = old **Z** → column 0 = $[0, 0, 1]^T$
- New **Y** (right) = old **X** → column 1 = $[1, 0, 0]^T$
- New **Z** (up) = old **-Y** → column 2 = $[0, -1, 0]^T$

Right-multiplying $\mathbf{R}_{WC}^{\text{RH}}$ by $\mathbf{R}_{ue \rightarrow cv}$ chains "world ← OpenCV" with "OpenCV ← UE" to get "world ← UE":

$$
\begin{aligned}
\mathbf{R}_{WU_{\text{cam}}}^{\text{RH}} &= \mathbf{R}_{WC}^{\text{RH}} \, \mathbf{R}_{ue \rightarrow cv} \\
\mathbf{t}_{WU_{\text{cam}}}^{\text{RH}} &= \mathbf{t}_{WC}
\end{aligned}
$$

After this step, the camera's local axes are realigned to UE convention (**X**=forward, **Y**=right, **Z**=up), but the world is **still right-handed**. The columns of $\mathbf{R}_{WU_{\text{cam}}}^{\text{RH}}$ express the UE camera axes in right-handed world coordinates.

### Step 2 — Handedness Flip (RH → LH)

Unreal Engine's world is **left-handed**, so reflect the Y-axis to change handedness. Define a reflection matrix:

$$
\mathbf{S} = \text{diag}(1, -1, 1) = \begin{bmatrix}
1 & 0 & 0 \\
0 & -1 & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

Apply it to both rotation and translation:

$$
\begin{aligned}
\mathbf{R}_{WU_{\text{cam}}}^{\text{LH}} &= \mathbf{S} \, \mathbf{R}_{WU_{\text{cam}}}^{\text{RH}} \\
\mathbf{t}_{WU_{\text{cam}}}^{\text{LH}} &= \mathbf{S} \, \mathbf{t}_{WU_{\text{cam}}}^{\text{RH}}
\end{aligned}
$$

**Why only a left-multiply by $\mathbf{S}$, not a sandwich $\mathbf{S} \mathbf{R} \mathbf{S}$?** The sandwich form is correct when an operator maps *within* a single coordinate system (both input and output live in the same space being flipped). Here, however, $\mathbf{R}_{WU_{\text{cam}}}^{\text{RH}}$ maps *between two different frames*: the UE camera frame on the right and the RH world on the left. Only the **world** side is changing (RH → LH via $\mathbf{S}$); the camera side was already converted to UE convention by $\mathbf{R}_{ue \rightarrow cv}$ in Step 1. So we apply $\mathbf{S}$ only on the left.

**Why $\mathbf{S} \mathbf{t}$?** The camera's physical position is the same, but the Y coordinate must be negated to express it in the left-handed world.

### Step 3: Final Result (Unreal Transform)

The resulting left-handed camera pose that can be supplied directly to Unreal Engine:

$$
\mathbf{T}_{WU_{\text{cam}}}^{\text{LH}} = \begin{bmatrix}
\mathbf{R}_{WU_{\text{cam}}}^{\text{LH}} & \mathbf{t}_{WU_{\text{cam}}}^{\text{LH}} \\
\mathbf{0}^T & 1
\end{bmatrix}
$$

where $\mathbf{R}_{WU_{\text{cam}}}^{\text{LH}} \in \mathbb{R}^{3 \times 3}$ is a proper rotation ($\det(\mathbf{R}_{WU_{\text{cam}}}^{\text{LH}}) = +1$) and $\mathbf{t}_{WU_{\text{cam}}}^{\text{LH}} \in \mathbb{R}^3$ is the camera position in the left-handed world.

Columns of $\mathbf{R}_{WU_{\text{cam}}}^{\text{LH}}$:

- Column 0 = camera **forward** (X) in UE world
- Column 1 = camera **right** (Y) in UE world
- Column 2 = camera **up** (Z) in UE world

Position is converted to centimeters for UE: $\mathbf{t}_{\text{cm}} = 100 \cdot \mathbf{t}_{WU_{\text{cam}}}^{\text{LH}}$

### Combined Formula

Combining both steps:

$$
\begin{aligned}
\mathbf{R}_{WU_{\text{cam}}}^{\text{LH}} &= \mathbf{S} \, \mathbf{R}_{WC}^{\text{RH}} \, \mathbf{R}_{ue \rightarrow cv} \\
\mathbf{t}_{WU_{\text{cam}}}^{\text{LH}} &= \mathbf{S} \, \mathbf{t}_{WC}
\end{aligned}
$$

where

$$
\mathbf{S} = \begin{bmatrix}
1 & 0 & 0 \\
0 & -1 & 0 \\
0 & 0 & 1
\end{bmatrix}
\qquad
\mathbf{R}_{ue \rightarrow cv} = \begin{bmatrix}
0 & 1 & 0 \\
0 & 0 & -1 \\
1 & 0 & 0
\end{bmatrix}
$$

### Numerical Example (E1)

**Input (OpenCV/RH):**

$$
\mathbf{R}_{WC}^{\text{RH}} = \begin{bmatrix}
-0.6363 & -0.6289 & -0.4467 \\
-0.1411 & 0.6642 & -0.7341 \\
0.7584 & -0.4041 & -0.5114
\end{bmatrix}
\qquad
\mathbf{t}_{WC} = \begin{bmatrix}
0.0220 \\
-0.1230 \\
0.0600
\end{bmatrix}
$$

$$\det(\mathbf{R}_{WC}^{\text{RH}}) = +1.0 \quad \text{(right-handed)}$$

**Output (Unreal/LH):**

$$
\mathbf{R}_{WU_{\text{cam}}}^{\text{LH}} = \begin{bmatrix}
-0.4467 & -0.6363 & 0.6289 \\
0.7341 & 0.1411 & 0.6642 \\
-0.5114 & 0.7584 & 0.4041
\end{bmatrix}
\qquad
\mathbf{t}_{WU_{\text{cam}}}^{\text{LH}} = \begin{bmatrix}
0.0220 \\
0.1230 \\
0.0600
\end{bmatrix}
$$

$$\det(\mathbf{R}_{WU_{\text{cam}}}^{\text{LH}}) = +1.0 \quad \text{(proper rotation)}$$

## Sanity Checks

The conversion preserves physical geometry:

| Check                              | Status |
|------------------------------------|--------|
| Inter-camera distances unchanged   | pass   |
| det(R) remains +1 (proper rotation)| pass   |
| Position Y-component negated       | pass   |

## Files

```
e1.json .. e4.json          Input: 3x4 [R|t] camera-to-world (RH, meters)
e1_ue.json .. e4_ue.json    Output: 4x4 transform + Euler angles (LH, meters & cm)
convert_and_visualize.py     Conversion + interactive side-by-side 3D visualization
cameras_side_by_side.png     Saved snapshot of the visualization
README.md                   This file
```

## Output JSON Format

Each `*_ue.json` contains:

```json
{
  "camera_name": "e1",
  "coordinate_system": "LEFT-HANDED (Unreal Engine)",
  "axes_convention": "X=Forward, Y=Right, Z=Up",
  "position_cm": { "x": 2.2, "y": 12.3, "z": 6.0 },
  "position_m":  { "x": 0.022, "y": 0.123, "z": 0.06 },
  "rotation_deg": { "roll": 119.83, "pitch": 30.76, "yaw": 121.32 },
  "rotation_matrix": [[ ... ]],
  "rotation_matrix_det": 1.0,
  "transform_4x4": [[ ... ]]
}
```

## Usage in Unreal Engine

```cpp
// From e1_ue.json:
FVector Location(2.2, 12.3, 6.0);           // position_cm
FRotator Rotation(30.76, 121.32, 119.83);    // FRotator(Pitch, Yaw, Roll)
Actor->SetActorLocationAndRotation(Location, Rotation);
```

Or use the 4x4 `transform_4x4` matrix directly as the actor's world transform.
