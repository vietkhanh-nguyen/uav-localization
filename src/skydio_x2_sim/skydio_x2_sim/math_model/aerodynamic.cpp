#include <pybind11/pybind11.h>
#include <pybind11/eigen.h> 
#include <pybind11/stl.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>

namespace py = pybind11;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;

// Định nghĩa AeroParams giống trong Python
struct AeroParams {
    double k_omega = 1.30e-7;   // N/RPM^2
    double k_z     = 2.55e-5;   // Vertical drag/thrust loss
    double k_h     = 3.39e-3;   // Horizontal lift
    double k_d     = 1.314e-5;  // Induced drag coeff  
    
    double l_x = 0.14;
    double l_y = 0.18;

    // Motor positions (4x3 matrix): [x, y, z] relative to CoM
    Eigen::Matrix<double, 4, 3> motor_positions;

    AeroParams() {
        // Initialize motor positions with Z offsets
        motor_positions <<  l_x, -l_y, 0.05,
                            l_x,  l_y, 0.05,
                           -l_x,  l_y, 0.08,
                           -l_x, -l_y, 0.08;
    }
};

// Hàm tiện ích: Chuyển Quaternion [w, x, y, z] (MuJoCo) sang Rotation Matrix
Matrix3d compute_rotation_from_quat(const Vector4d& q_mujoco) {
    // Eigen Quaternion constructor: (w, x, y, z)
    Quaterniond q(q_mujoco(0), q_mujoco(1), q_mujoco(2), q_mujoco(3));
    q.normalize(); 
    return q.toRotationMatrix();
}

class AerodynamicModel {
public:
    AeroParams pms;
    Matrix3d P; // Projection Matrix (diag(1, 1, 0))

    AerodynamicModel() {
        // Initialize Projection Matrix P
        P << 1, 0, 0,
             0, 1, 0,
             0, 0, 0;
    }

    AerodynamicModel(const AeroParams& params) : pms(params) {
        P << 1, 0, 0,
             0, 1, 0,
             0, 0, 0;
    }

    Eigen::Matrix<double, 6, 1> cal_induced_drag_wrench(
        const Vector4d& quat, 
        const Vector4d& omega_m, 
        const Vector3d& v_body
    ) {

        Matrix3d Rot = compute_rotation_from_quat(quat);

        double sum_omega = omega_m.sum();
        Vector3d D_body = -pms.k_d * sum_omega * (P * v_body);
        Vector3d D_world = Rot * D_body;


        Vector3d D_M_body = Vector3d::Zero();
        for (int i = 0; i < 4; ++i) {
            Vector3d f_drag_i = -pms.k_d * omega_m(i) * (P * v_body);

            Vector3d r_i = pms.motor_positions.row(i);

            D_M_body += r_i.cross(f_drag_i);
        }

        Vector3d D_M_world = Rot * D_M_body;

        Eigen::Matrix<double, 6, 1> wrench;
        wrench << D_world, D_M_world;

        return wrench;
    }

    Vector4d cal_thrust(const Vector4d& omega_m, const Vector3d& v_body) {
        double v_horiz_sq = v_body(0)*v_body(0) + v_body(1)*v_body(1);
        
        // Công thức Refined Thrust Model
        Vector4d T = pms.k_omega * omega_m.array().square() 
                   - pms.k_z * omega_m.array() * v_body(2) 
                   + pms.k_h * v_horiz_sq;
        
        return T;
    }
};

// ------------------------------------------------------------------
PYBIND11_MODULE(aerodynamics_cpp, m) {
    m.doc() = "C++ Aerodynamics Module for Drone Simulation";

    py::class_<AeroParams>(m, "AeroParams")
        .def(py::init<>())
        .def_readwrite("k_omega", &AeroParams::k_omega)
        .def_readwrite("k_z", &AeroParams::k_z)
        .def_readwrite("k_h", &AeroParams::k_h)
        .def_readwrite("k_d", &AeroParams::k_d)
        .def_readwrite("l_x", &AeroParams::l_x)
        .def_readwrite("l_y", &AeroParams::l_y)
        .def_readwrite("motor_positions", &AeroParams::motor_positions);

    py::class_<AerodynamicModel>(m, "AerodynamicModel")
        .def(py::init<>())
        .def(py::init<const AeroParams&>()) 
        .def_readwrite("pms", &AerodynamicModel::pms)
        .def("cal_induced_drag_wrench", &AerodynamicModel::cal_induced_drag_wrench,
             "Calculate Drag Force & Moment in World Frame. Input: Quat[w,x,y,z], RPM, V_body",
             py::arg("quat"), py::arg("omega_m"), py::arg("v_body"))
        .def("cal_thrust", &AerodynamicModel::cal_thrust,
             "Calculate Thrust per motor. Input: RPM, V_body",
             py::arg("omega_m"), py::arg("v_body"));
}