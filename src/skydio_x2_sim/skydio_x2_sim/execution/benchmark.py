import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import numpy as np
from math_model import drone_dynamics_cpp  # This imports your compiled C++ module
from math_model.drone_model import QuadcopterDynamicModel # Import the old Python class for comparison
from math_model.model_pms import MyX2Params, AeroParams

def test_performance():
    # --- 1. Setup Data ---
    # Random inputs for testing
    # angle   = np.array([0.1, -0.05, 0.2])        # Roll, Pitch, Yaw (rad)
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    V_world = np.array([2.5, -1.0, 0.5])         # Vx, Vy, Vz (m/s)
    Omega   = np.array([0.1, 0.2, -0.1])         # P, Q, R (rad/s)
    omega_m = np.array([5500.0, 5400.0, 5600.0, 5500.0]) # Motor RPM

    # --- 2. Initialize Models ---
    print("Initializing models...")
    
    # Python Model
    params = MyX2Params()
    py_model = QuadcopterDynamicModel(params)
    
    # C++ Model (The params are hardcoded inside C++ struct as per previous step)
    cpp_model = drone_dynamics_cpp.QuadcopterDynamicModel()

    cpp_model.pms.m = params.m
    cpp_model.pms.J = params.J
    cpp_model.pms.Jinv = params.Jinv
    cpp_model.pms.motor_positions = params.motor_positions
    cpp_model.pms.mixing_matrix = params.mixing_matrix
    cpp_model.pms.motor_positions = params.motor_positions

    # --- 3. Verify Correctness (Compare outputs) ---
    print("\n--- Correctness Check ---")
    
    # Run Python
    res_py = py_model.forward_dynamics(np.array([quat[1], quat[2], quat[3], quat[0]]), V_world, Omega, omega_m)
    
    # Run C++
    res_cpp = cpp_model.forward_dynamics(quat, V_world, Omega, omega_m)
    
    print(f"Python Result (first 3): {res_py[:3]}")
    print(f"C++ Result    (first 3): {res_cpp[:3]}")
    
    # Check difference
    diff = np.linalg.norm(res_py - res_cpp)
    if diff < 1e-9:
        print("✅ SUCCESS: Results match perfectly!")
    else:
        print(f"❌ WARNING: Results differ by {diff}. Check constants/equations.")

    # --- 4. Benchmark Speed ---
    print("\n--- Speed Benchmark (100,000 iterations) ---")
    iterations = 100000
    
    # Measure Python
    start = time.time()
    for _ in range(iterations):
        py_model.forward_dynamics(quat, V_world, Omega, omega_m)
    py_time = time.time() - start
    print(f"Python Time: {py_time:.4f} sec")

    # Measure C++
    start = time.time()
    for _ in range(iterations):
        cpp_model.forward_dynamics(quat, V_world, Omega, omega_m)
    cpp_time = time.time() - start
    print(f"C++ Time:    {cpp_time:.4f} sec")

    # Conclusion
    print(f"🚀 Speedup: {py_time / cpp_time:.2f}x faster")

if __name__ == "__main__":
    test_performance()