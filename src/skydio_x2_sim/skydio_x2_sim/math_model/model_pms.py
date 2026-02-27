import numpy as np

class MyX2Params:
    J = np.array([
    [ 0.0366517,  0.0, 0.0021],
    [ 0.0, 0.0254117,  0.0],
    [0.0021, 0., 0.060528],
    ])
    motor_positions = np.array([
        [ 0.14, -0.18, 0.08], 
        [ 0.14,  0.18, 0.08], 
        [-0.14,  0.18, 0.05], 
        [-0.14, -0.18, 0.05]  
    ])
    
    Jinv = np.linalg.inv(J)
    g = 9.8
    k_omega = 1.30e-7   
    k_z     = 2.55e-5   
    k_h     = 3.39e-3   
    k_d     = 1.314e-5  
    k_m = 0.0201
    mixing_matrix = np.array([
            [ 1,  1,  1,  1],                   
            [ -0.18, 0.18, 0.18,  -0.18],        
            [ -0.14,  -0.14, 0.14, 0.14],        
            [-k_m,  k_m, -k_m,  k_m]            
    ])
    m = 1.326

class X2Params:
    J = np.array([
    [ 0.0366517,  0.0, -0.0021],
    [ 0.0, 0.0254117,  0.0],
    [-0.0021, 0., 0.060528],
    ])
    l_x = 0.14
    l_y = 0.18
    motor_positions = np.array([
        [ -l_x, -l_y, 0.05],
        [ -l_x,  l_y, 0.05],
        [l_x,  l_y, 0.08],
        [l_x, -l_y, 0.08]
    ])
    
    Jinv = np.linalg.inv(J)
    g = 9.8
    k_omega = 1.30e-7   # N/RPM^2
    k_z     = 2.55e-5   # Vertical drag/thrust loss
    k_h     = 3.39e-3   # Horizontal lift
    k_d     = 1.314e-5  # Induced drag coeff 
    k_m = 0.0201
    mixing_matrix = np.array([
        [1, 1, 1, 1],
        [-l_y, l_y, l_y, -l_y],
        [l_x, l_x, -l_x, -l_x],
        [-k_m, k_m, -k_m, k_m]
    ])
    m = 1.325

class AeroParams:
    k_omega = 1.30e-7   # N/RPM^2
    k_z     = 2.55e-5   # Vertical drag/thrust loss
    k_h     = 3.39e-3   # Horizontal lift
    k_d     = 1.314e-5  # Induced drag coeff 
    l_x = 0.14
    l_y = 0.18
    motor_positions = np.array([
        [ l_x, -l_y, 0.05],
        [ l_x,  l_y, 0.05],
        [-l_x,  l_y, 0.08],
        [-l_x, -l_y, 0.08]
    ])