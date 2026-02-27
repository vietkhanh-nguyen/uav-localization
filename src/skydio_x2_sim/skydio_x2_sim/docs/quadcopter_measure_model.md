# Dynamics & Aerodynamics Formulas Summary
**Source:** Svacha et al., 2017 (ICUAS)

## 1. Notation & Coordinate Systems
* **Frames:** World Frame $W$ (North-West-Up), Body Frame $B$ (Forward-Left-Up).
* **State:** $v$ (Velocity in $W$), $\Omega$ (Angular velocity in $B$), $q$ (quaternion rotation in $W$), $R = R_{WB}$ (Rotation $B \to W$).

## 2. Quadrotor Dynamics and predictive model

### Translational Dynamics in world frame

$$\dot{v} = \frac{F}{m} R e_3 - g e_3 $$

### Rotational Dynamics in body frame
Includes gyroscopic effects, external moments, and drag-induced moment ($h$ is rotor height):
$$\dot{\Omega} = J^{-1} (-\Omega \times J \Omega + M)$$

### Accelerometer Measurement Model
Measures specific force (Thrust + Drag) in Body Frame:
$$a_{IMU} = R^T (\dot{v} + g e_3) + b_a$$
$$\Omega_{IMU} = \Omega + b_\Omega$$

### Model predict base on IMU and control input
$$ 
\begin{cases}
    \dot{p} &= v \\
    \dot{v} &= R (a_{IMU} - b_a) - g e_3 \\
    \dot{q} &= \dfrac{1}{2} q \otimes \begin{bmatrix} 0 \\ \Omega_{IMU} - b_\Omega \end{bmatrix}
\end{cases}
$$

