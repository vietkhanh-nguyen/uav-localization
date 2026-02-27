import mujoco as mj
import numpy as np

def analyze_drone_physical_params(model, body_name="x2"):
    """
    Analyzes and prints physical parameters (Mass, CoM, Inertia, Motors) 
    of a specific body in a MuJoCo model.
    
    Args:
        model: The mujoco.MjModel object.
        body_name (str): The name of the body to analyze (e.g., "x2").
    """
    print(f"\n{'='*20} ANALYZING BODY: '{body_name}' {'='*20}")
    
    # 1. Get Body ID
    try:
        body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, body_name)
        if body_id == -1:
            print(f"Error: Body '{body_name}' not found in model.")
            return
    except Exception as e:
        print(f"Error finding body: {e}")
        return

    # 2. Total Mass (Subtree mass is usually what we want for a free-floating drone)
    total_mass = model.body_subtreemass[body_id]
    print(f"Total Mass (subtree): {total_mass:.4f} kg")

    # 3. Center of Mass (CoM) Position
    # body_ipos is the position of the inertial frame relative to the body frame
    com_pos = model.body_ipos[body_id]
    print(f"CoM Position (relative to body frame): {com_pos}")

    # 4. Inertia Matrix Calculation
    # MuJoCo stores inertia as a diagonal vector (inertial_diag) and a quaternion (inertial_quat)
    # representing the rotation of the inertial frame.
    
    # Get diagonal inertia
    inertia_diag = model.body_inertia[body_id]
    I_diag_matrix = np.diag(inertia_diag)
    
    # Get orientation of inertial frame
    iquat = model.body_iquat[body_id]
    print(f"Inertial Quaternion (w, x, y, z): {iquat}")
    
    # Convert Quaternion to Rotation Matrix
    R = np.zeros(9)
    mj.mju_quat2Mat(R, iquat)
    R = R.reshape(3, 3)
    
    # Compute Full Inertia Matrix in Body Frame: I_body = R * I_diag * R.T
    I_full = R @ I_diag_matrix @ R.T
    
    print("\n--- Full 3x3 Inertia Matrix (in Body Frame) ---")
    print(I_full)
    
    print(f"\n--- Inertia Check ---")
    print(f"I_xx: {I_full[0, 0]:.8f} | I_xy: {I_full[0, 1]:.8f} | I_xz: {I_full[0, 2]:.8f}")
    print(f"I_yx: {I_full[1, 0]:.8f} | I_yy: {I_full[1, 1]:.8f} | I_yz: {I_full[1, 2]:.8f}")
    print(f"I_zx: {I_full[2, 0]:.8f} | I_zy: {I_full[2, 1]:.8f} | I_zz: {I_full[2, 2]:.8f}")

    if not (np.allclose(iquat, [1, 0, 0, 0]) or np.allclose(iquat, [0, 1, 0, 0])):
        print("=> NOTE: Inertial frame is ROTATED (Off-diagonals are non-zero).")

    # 5. Actuators / Motors Inspection
    print("\n--- Actuator/Motor Positions ---")
    print("(Assuming actuators are attached via Sites relative to this body)")
    
    for i in range(model.nu):
        # Check if actuator is defined by a Site
        if model.actuator_trntype[i] == mj.mjtTrn.mjTRN_SITE:
            site_id = model.actuator_trnid[i, 0]
            site_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_SITE, site_id)
            site_pos = model.site_pos[site_id]
            
            # Identify which gear/gain
            gear = model.actuator_gear[i]
            
            # Simple check: Is this motor relevant? (Optional: Filter by name logic)
            print(f"Actuator {i} ('{site_name}'):")
            print(f"  -> Local Pos:  X={site_pos[0]:.4f}, Y={site_pos[1]:.4f}, Z={site_pos[2]:.4f}")
            print(f"  -> Gear (Cf/Ct): {np.array2string(gear, precision=4, separator=', ')}")

        elif model.actuator_trntype[i] == mj.mjtTrn.mjTRN_JOINT:
            print(f"Actuator {i} is Joint-based (skipped for drone site check).")

    print("="*60 + "\n")

def print_drone_physical_params(model, body_name="x2"):
    """
    Analyzes physical parameters of a specific body in a MuJoCo model
    and prints a Python class definition for direct copy-pasting.
    Uses actuator gear to automatically determine Mixing Matrix.
    """
    print(f"\n{'='*20} ANALYZING BODY: '{body_name}' {'='*20}")
    
    # 1. Get Body ID
    try:
        body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, body_name)
        if body_id == -1:
            print(f"Error: Body '{body_name}' not found in model.")
            return
    except Exception as e:
        print(f"Error finding body: {e}")
        return

    # --- DATA COLLECTION ---

    # 2. Total Mass
    total_mass = model.body_subtreemass[body_id]

    # 3. Inertia Calculation
    inertia_diag = model.body_inertia[body_id]
    I_diag_matrix = np.diag(inertia_diag)
    iquat = model.body_iquat[body_id]
    
    # Convert Quaternion to Rotation Matrix
    R = np.zeros(9)
    mj.mju_quat2Mat(R, iquat)
    R = R.reshape(3, 3)
    
    # Full Inertia Matrix in Body Frame: I_body = R * I_diag * R.T
    I_full = R @ I_diag_matrix @ R.T

    # 4. Actuators / Motor Positions / Gears
    motor_positions = []
    motor_gears = []
    
    # Iterate through actuators to find those linked to sites
    for i in range(model.nu):
        # We only care about site transmission (standard for multicopters)
        if model.actuator_trntype[i] == mj.mjtTrn.mjTRN_SITE:
            site_id = model.actuator_trnid[i, 0]
            site_pos = model.site_pos[site_id]
            
            # Get the gear (force/torque scales)
            # gear[2] is usually Thrust scale (z-force)
            # gear[5] is usually Yaw Torque scale (z-torque)
            gear = model.actuator_gear[i]
            
            motor_positions.append(site_pos)
            motor_gears.append(gear)

    motor_positions = np.array(motor_positions)
    motor_gears = np.array(motor_gears)

    # --- PRINTING THE PYTHON CLASS ---
    
    class_name = body_name.capitalize() + "Params"
    
    print("\n" + "#"*30)
    print("### COPY THE CODE BELOW ###")
    print("#"*30 + "\n")

    print(f"class {class_name}:")
    print(f"    m = {total_mass:.4f}")
    print(f"    g = 9.8")
    
    # Extract k_m from the first motor's gear (magnitude of z-torque)
    # gear index 5 is Z-torque
    k_m_val = 0.0
    if len(motor_gears) > 0:
        k_m_val = abs(motor_gears[0][5])
    
    print(f"    k_m = {k_m_val:.6f}  # Auto-detected from MuJoCo gear")
    
    # Print Inertia Matrix J
    print("\n    J = np.array([")
    for row in I_full:
        print(f"        [{row[0]:.8f}, {row[1]:.8f}, {row[2]:.8f}],")
    print("    ])")
    
    print("\n    Jinv = np.linalg.inv(J)")

    # Print Motor Positions
    print("\n    motor_positions = np.array([")
    for pos in motor_positions:
        print(f"        [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}],")
    print("    ])")

    # Construct Mixing Matrix automatically
    if len(motor_positions) == 4:
        # 1. Thrust Row: usually gear[2] (Z-force scale)
        thrust_coeffs = motor_gears[:, 2] 
        
        # 2. Roll Row: Torque X = y * Fz 
        # (Right-hand rule: Force Up at +y creates +x Torque)
        roll_coeffs = motor_positions[:, 1] * thrust_coeffs
        
        # 3. Pitch Row: Torque Y = -x * Fz 
        # (Right-hand rule: Force Up at +x creates -y Torque)
        pitch_coeffs = -motor_positions[:, 0] * thrust_coeffs
        
        # 4. Yaw Row: Directly read from gear[5]
        yaw_coeffs = motor_gears[:, 5]
        
        print("\n    # Mixing Matrix (Auto-generated from Body Frame & Actuator Gears)")
        print("    # Row 1: Thrust (Z)")
        print("    # Row 2: Roll (Torque X = y * Fz)")
        print("    # Row 3: Pitch (Torque Y = -x * Fz)")
        print("    # Row 4: Yaw (Direct from gear[5])")
        
        print("    mixing_matrix = np.array([")
        
        # Helper to format list
        def fmt(arr): return ", ".join([f"{x:.4f}" for x in arr])

        print(f"        [{fmt(thrust_coeffs)}],")
        print(f"        [{fmt(roll_coeffs)}],")
        print(f"        [{fmt(pitch_coeffs)}],")
        print(f"        [{fmt(yaw_coeffs)}]")
        
        print("    ])")
    else:
        print(f"\n    # Warning: Found {len(motor_positions)} motors. Mixing matrix generation skipped.")

    print("\n" + "#"*30)


if __name__ == "__main__":
    try:
        # Load your specific model
        model = mj.MjModel.from_xml_path("mjcf/my_x2.xml") 
        
        # Run the analysis
        analyze_drone_physical_params(model, body_name="x2")
        print_drone_physical_params(model, body_name="x2")
        
    except Exception as e:
        print(f"Cannot load model for standalone test: {e}")