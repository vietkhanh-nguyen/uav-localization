# Dynamics & Aerodynamics Formulas Summary
**Source:** Svacha et al., 2017 (ICUAS)

## 1. Notation & Coordinate Systems
* **Frames:** World Frame $W$ (North-West-Up), Body Frame $B$ (Forward-Left-Up).
* **State:** $V$ (Velocity in $W$), $\Omega$ (Angular velocity in $B$), $R$ (Rotation $B \to W$).
* **Inputs:** $\omega_i$ (Motor speed), $\omega_s = \sum \omega_i$ (Sum of motor speeds).

## 2. Aerodynamic Models

### Linear Induced Drag Model
Modeled as linear in body $x-y$ velocity:
$$D = -k_d \omega_s R P R^T V$$
**Where:**
* $P = \text{diag}(1, 1, 0)$ is the projection matrix.
* $k_d$ is the drag coefficient.

### Induced Velocity Model
Approximated as linear function of vertical velocity and motor speed:
$$v_i(\omega_i, V_{zi}) = a \omega_i - b V_{zi}$$

### Refined Thrust Model
Lumped parameter model derived from BEMT (Blade Element Momentum Theory):
$$T_i = k_\omega \omega_i^2 - k_z V_{zi} \omega_i + k_h V_{hi}^2$$
**Where:**
* $k_\omega$: Standard thrust coefficient.
* $k_z$: Coefficient for vertical velocity effect.
* $k_h$: Coefficient for horizontal velocity effect.
* $V_{zi}, V_{hi}$: Vertical and horizontal velocity of $i^{th}$ propeller in Body Frame.

---

## 3. Quadrotor Dynamics

### Translational Dynamics
Includes gravity and induced drag:
$$\dot{V} = \frac{F}{m} R e_3 - g e_3 - \frac{k_d}{m} \omega_s R P R^T V$$

### Rotational Dynamics
Includes gyroscopic effects, external moments, and drag-induced moment ($h$ is rotor height):
$$\dot{\Omega} = J^{-1} (\Omega \times J \Omega - M + \sum_{i = 1}^4 (k_d \omega_s R P R^T V) \times \vec{l}_i)$$

### Accelerometer Measurement Model
Measures specific force (Thrust + Drag) in Body Frame:
$$a_{IMU} = R^T (\dot{V} + g e_3) + b_a$$
**For drag ID (x-axis, forward flight):**
$$a_{IMUx} = -\frac{k_d}{m} \omega_s V_x \cos\theta + b_{ax}$$

---

## 4. System Identification Equations

### Thrust Coefficients ID
Summing refined thrust model over all motors:
$$\sum_{i=1}^{4} T_i = k_\omega \sum_{i=1}^{4} \omega_i^2 - k_z V_z \sum_{i=1}^{4} \omega_i + 4 k_h V_h^2$$
* **Vertical flight:** $\sum T_i \approx mg$.
* **Horizontal flight:** $\sum T_i \approx m \cdot a_{IMUz}$.

### Mixing Matrix (Standard + Config)
*Note: This specific matrix is for the paper's '+' configuration, not the X2.*
$$
\begin{bmatrix}
F \\ M_x \\ M_y \\ M_z
\end{bmatrix} = 
\begin{bmatrix}
1 & 1 & 1 & 1 \\
0 & L & 0 & -L \\
-L & 0 & L & 0 \\
k_M & -k_M & k_M & -k_M
\end{bmatrix}
\begin{bmatrix}
T_1 \\ T_2 \\ T_3 \\ T_4
\end{bmatrix}
$$

---

## 5. Control Compensation

### Drag Compensation (Attitude Command)
Find $R_{cmd}$ and $F_{cmd}$ such that:
$$F_{cmd} R_{cmd} e_3 + k_d \omega_s R_{cmd} P R_{cmd}^T V = \alpha$$
**Where:** $\alpha = m(a_{des} - K_v e_v - K_x e_x + g e_3)$ is the desired force vector.

### Thrust Compensation
Solve for $\omega_i$ (closed form quadratic) given required $T_i$:
$$k_\omega \omega_i^2 - k_z V_z \omega_i + (k_h V_h^2 - T_i) = 0$$

---
---

# Skydio X2 Simulation Parameters

**Reference Paper:** Svacha et al., "Improving Quadrotor Trajectory Tracking by Compensating for Aerodynamic Effects" (2017).
**Model Source:** Skydio X2 Mujoco XML (User Provided).

## 1. Rigid Body & Geometry Parameters
*Derived from `x2.xml` and Mujoco mass/inertia calculation.*

| Parameter | Symbol | Value (Skydio X2) | Unit | Description |
| :--- | :---: | :--- | :--- | :--- |
| **Total Mass** | $m$ | `1.325` | $kg$ | Total mass of the drone. |
| **Inertia Tensor** | $J$ | `[0.0607, 0.0365, 0.0254]` | $kg \cdot m^2$ | Diagonal body inertia $(I_{xx}, I_{yy}, I_{zz})$. |
| **Arm Length (X)** | $l_x$ | `0.14` | $m$ | Distance from CoM to motor along body x-axis. |
| **Arm Length (Y)** | $l_y$ | `0.18` | $m$ | Distance from CoM to motor along body y-axis. |
| **Propeller Radius** | $r$ | `0.13` | $m$ | Derived from geom size in XML. |
| **Rotor Height** | $h$ | `0.05` | $m$ | Vertical distance of prop plane from CoM (used in Eq. 21). |

---

## 2. Control Allocation Coefficients (Mixing)
*Derived from `x2.xml` actuator gears.*

| Parameter | Symbol | Value | Source | Description |
| :--- | :---: | :--- | :--- | :--- |
| **Torque/Thrust Ratio** | $k_M$ | `0.0201` | XML `gear` | Relates motor thrust to drag moment: $M_i = k_M T_i$. |
| **Thrust Vector** | - | `[0, 0, 1]` | XML `gear` | Actuators apply force purely in body Z-axis. |

---

## 3. Aerodynamic Coefficients (The "Refined Model")
*These values define the relationship between RPM, Velocity, and Thrust/Drag. Since `x2.xml` uses generic physics, these values are taken from the paper as initial estimates for the "Refined Thrust Model" and "Linear Drag Model".*

**Important Note:** The units below are based on the paper's regression which uses **RPM** for motor speed, not rad/s.

### A. Thrust Model
*Formula:* $T_i = k_\omega \omega_i^2 - k_z V_{zi} \omega_i + k_h V_{hi}^2$.

| Parameter | Symbol | Value (Paper Ref) | Value (X2 Est.) | Unit | Description |
| :--- | :---: | :--- | :--- | :--- | :--- |
| **Thrust Coeff** | $k_\omega$ | $8.00 \times 10^{-8}$ | **$1.30 \times 10^{-7}$*** | $N / RPM^2$ | Basic static thrust coefficient ($T \approx k_\omega \omega^2$). |
| **Vertical Coeff** | $k_z$ | $2.55 \times 10^{-5}$ | $2.55 \times 10^{-5}$ | $\frac{N \cdot s}{m \cdot RPM}$ | Loss of thrust due to vertical velocity (descent/ascent). |
| **Horizontal Coeff**| $k_h$ | $3.39 \times 10^{-3}$ | $3.39 \times 10^{-3}$ | $\frac{N \cdot s^2}{m^2}$ | Gain of thrust due to horizontal airspeed (translational lift). |

*\*Note: The X2 Estimate for $k_\omega$ is calculated based on the X2 mass (1.325kg) requiring ~5000 RPM to hover, whereas the Paper Ref is for a lighter AscTec Hummingbird.*

### B. Drag Model
*Formula:* $D = -k_d \omega_s R P R^T V$.

| Parameter | Symbol | Value (Paper Ref) | Unit | Description |
| :--- | :---: | :--- | :--- | :--- |
| **Drag Coeff** | $k_d$ | $1.314 \times 10^{-5}$ | $\frac{N \cdot s}{m \cdot RPM}$ | Lumped induced drag coefficient. Depends on total motor speed $\omega_s$. |

---

## 4. Simulation Implementation Checklist

To simulate the **Refined Model** in Mujoco using Python:

1.  **Inputs:** Desired RPM ($\omega_{des}$) for 4 motors.
2.  **State:** Get Body Velocity $V_b = [v_x, v_y, v_z]$.
3.  **Compute Thrust ($T_i$):**
    $$T_i = \text{max}(0, k_\omega \omega_{des}^2 - k_z v_z \omega_{des} + k_h (v_x^2 + v_y^2))$$
4.  **Send to Mujoco:**
    * `data.ctrl[:] = [T_1, T_2, T_3, T_4]`
    * *Mujoco automatically applies Torque* $M_z = \pm 0.0201 \times T_i$ based on XML.
5.  **Compute Induced Drag Force ($D$):**
    * $\omega_s = \sum \omega_{des}$
    * $D_{body} = -k_d \cdot \omega_s \cdot [v_x, v_y, 0]$
    * Apply this force to the center of mass using `data.xfrc_applied`.