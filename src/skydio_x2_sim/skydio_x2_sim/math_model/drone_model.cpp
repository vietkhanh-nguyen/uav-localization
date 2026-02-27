#include <pybind11/pybind11.h>
#include <pybind11/eigen.h> 
#include <pybind11/stl.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>
#include <algorithm> 

namespace py = pybind11;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;

struct X2Params {
    double l_x = 0.14;
    double l_y = 0.18;
    double g = 9.8;
    double m = 1.325;
    
    double k_omega = 1.30e-7;   
    double k_z     = 2.55e-5;   
    double k_h     = 3.39e-3;   
    double k_d     = 1.314e-5;  
    double k_m     = 0.0201;

    Matrix3d J;
    Matrix3d Jinv;
    Matrix4d mixing_matrix;
    
    Eigen::Matrix<double, 4, 3> motor_positions;

    X2Params() {
        J << 0.0366517, 0.0, -0.0021,
             0.0, 0.0254117, 0.0,
            -0.0021, 0.0, 0.060528;
        Jinv = J.inverse();

        motor_positions <<  l_x, -l_y, 0.05,
                            l_x,  l_y, 0.05,
                           -l_x,  l_y, 0.08,
                           -l_x, -l_y, 0.08;

        mixing_matrix << 1.0, 1.0, 1.0, 1.0,
                        -l_y, l_y, l_y, -l_y,
                         l_x, l_x, -l_x, -l_x,
                        -k_m, k_m, -k_m, k_m;
    }
};

Matrix3d compute_rotation_from_quat(const Vector4d& q_mujoco) {
    Quaterniond q(q_mujoco(0), q_mujoco(1), q_mujoco(2), q_mujoco(3));
    q.normalize(); 
    return q.toRotationMatrix();
}

class QuadcopterDynamicModel {
public:
    X2Params pms;
    Matrix3d P; 

    QuadcopterDynamicModel() {
        P << 1, 0, 0, 
             0, 1, 0, 
             0, 0, 0;
    }

    Vector4d cal_thrust(const Vector3d& V_body, const Vector4d& omega_m) {
        Vector4d T;
        double v_horiz_sq = V_body(0)*V_body(0) + V_body(1)*V_body(1);
        T = pms.k_omega * omega_m.array().square() 
            - pms.k_z * omega_m.array() * V_body(2) 
            + pms.k_h * v_horiz_sq;
        return T;
    }

    Eigen::Matrix<double, 6, 1> forward_dynamics(
        const Vector4d& quat, 
        const Vector3d& V_body, 
        const Vector3d& Omega, 
        const Vector4d& omega_m
    ) {
        Matrix3d Rot = compute_rotation_from_quat(quat);

        Vector4d T = cal_thrust(V_body, omega_m);
        Vector4d gen_force = pms.mixing_matrix * T;

        double F_thrust_scalar = gen_force(0);
        Vector3d M_control = gen_force.tail<3>();

        Vector3d F_thrust_body(0, 0, F_thrust_scalar);
        Vector3d F_thrust_world = Rot * F_thrust_body;
        
        Vector3d g_vec(0, 0, pms.g);

        Vector3d dV_world = F_thrust_world / pms.m - g_vec;

        Vector3d torque_total = M_control - Omega.cross(pms.J * Omega);
        Vector3d dOmega_body = pms.Jinv * torque_total;
        Vector3d dOmega_world = Rot * dOmega_body;

        Eigen::Matrix<double, 6, 1> state_dot;
        state_dot << dV_world, dOmega_world;
        return state_dot;
    }
 
    Eigen::Matrix<double, 6, 1> forward_dynamics_aero(
        const Vector4d& quat, 
        const Vector3d& v_body, 
        const Vector3d& Omega, 
        const Vector4d& omega_m
    ) {
        Matrix3d Rot = compute_rotation_from_quat(quat);
        Vector4d T = cal_thrust(v_body, omega_m);
        Vector4d gen_force = pms.mixing_matrix * T;
        double F_thrust_scalar = gen_force(0);
        Vector3d M_control = gen_force.tail<3>();
        double sum_omega = omega_m.sum();
        Vector3d D_body = -pms.k_d * sum_omega * (P * v_body);
        Vector3d D_M_body = Vector3d::Zero();
        for (int i = 0; i < 4; ++i) {
            Vector3d f_drag_i = -pms.k_d * omega_m(i) * (P * v_body);
            Vector3d r_i = pms.motor_positions.row(i); 
            D_M_body += r_i.cross(f_drag_i);
        }
        Vector3d F_thrust_body(0, 0, F_thrust_scalar);
        Vector3d F_thrust_world = Rot * F_thrust_body;
        Vector3d D_world = Rot * D_body;
        Vector3d g_vec(0, 0, pms.g);
        Vector3d dV_world = (F_thrust_world + D_world) / pms.m - g_vec;
        Vector3d torque_total = M_control + D_M_body - Omega.cross(pms.J * Omega);
        Vector3d dOmega_body = pms.Jinv * torque_total;
        Vector3d dOmega_world = Rot * dOmega_body;

        Eigen::Matrix<double, 6, 1> state_dot;
        state_dot << dV_world, dOmega_world;
        return state_dot;
    }

    double solve_motor_speed(const Vector3d& v_body, double T_i) {
        double a = pms.k_omega;
        double b = -pms.k_z * v_body(2);
        double c = pms.k_h * (v_body(0)*v_body(0) + v_body(1)*v_body(1)) - T_i;

        double delta = b*b - 4*a*c;
        
        if (delta < 0) delta = 0.0;

        return (-b + std::sqrt(delta)) / (2 * a);
    }

    double solve_motor_speed_simple(double T_i) {
        if (T_i < 0) T_i = 0.0;
        return std::sqrt(T_i / pms.k_omega);
    }
};

PYBIND11_MODULE(drone_dynamics_cpp, m) {
    m.doc() = "Updated C++ Drone Dynamics";

    m.def("compute_rotation_from_quat", &compute_rotation_from_quat, 
          "Convert MuJoCo Quaternion [w,x,y,z] to 3x3 Rotation Matrix",
          py::arg("q_mujoco"));

    py::class_<X2Params>(m, "X2Params")
        .def(py::init<>())
        .def_readwrite("l_x", &X2Params::l_x)
        .def_readwrite("l_y", &X2Params::l_y)
        .def_readwrite("g", &X2Params::g)
        .def_readwrite("m", &X2Params::m)
        .def_readwrite("k_omega", &X2Params::k_omega)
        .def_readwrite("k_z", &X2Params::k_z)
        .def_readwrite("k_h", &X2Params::k_h)
        .def_readwrite("k_d", &X2Params::k_d)
        .def_readwrite("mixing_matrix", &X2Params::mixing_matrix)
        .def_readwrite("J", &X2Params::J)
        .def_readwrite("Jinv", &X2Params::Jinv)
        .def_readwrite("motor_positions", &X2Params::motor_positions);

    py::class_<QuadcopterDynamicModel>(m, "QuadcopterDynamicModel")
        .def(py::init<>())
        .def_readwrite("pms", &QuadcopterDynamicModel::pms)
        .def("cal_thrust", &QuadcopterDynamicModel::cal_thrust, 
             py::arg("V_body"), py::arg("omega_m"))
        
        .def("forward_dynamics", &QuadcopterDynamicModel::forward_dynamics,
             py::arg("quat"), py::arg("V_body"), py::arg("Omega"), py::arg("omega_m"))
        
        .def("forward_dynamics_aero", &QuadcopterDynamicModel::forward_dynamics_aero,
             py::arg("quat"), py::arg("v_body"), py::arg("Omega"), py::arg("omega_m"))
        
        .def("solve_motor_speed", &QuadcopterDynamicModel::solve_motor_speed,
             py::arg("v_body"), py::arg("T_i"))
             
        .def("solve_motor_speed_simple", &QuadcopterDynamicModel::solve_motor_speed_simple,
             py::arg("T_i"));
}