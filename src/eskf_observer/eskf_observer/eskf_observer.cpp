#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace py = pybind11;
using namespace Eigen;

class ErrorStateKF {
public:
    double dt;
    Vector3d p, v, ab, wb, g;
    Quaterniond q;
    Matrix<double, 15, 15> P;

    ErrorStateKF(double dt_in) : dt(dt_in) {
        p.setZero();
        v.setZero();
        q.setIdentity();
        ab.setZero();
        wb.setZero();
        g << 0, 0, -9.81;
        P = Matrix<double, 15, 15>::Identity() * 0.01;
        P.block<3, 3>(9, 9) *= 0.01;
        P.block<3, 3>(12, 12) *= 0.01;
    }

    Matrix3d skew(const Vector3d& v) {
        Matrix3d m;
        m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
        return m;
    }

    void predict(const Vector3d& am, const Vector3d& wm, 
                 double sigma_an, double sigma_wn, double sigma_aw, double sigma_ww) {
        Matrix3d R_mat = q.toRotationMatrix();
        Vector3d acc_body = am - ab;
        Vector3d omega_body = wm - wb;

        Vector3d acc_world = R_mat * acc_body + g;
        p += v * dt + 0.5 * acc_world * dt * dt;
        v += acc_world * dt;

        Vector3d angle_step = omega_body * dt;
        double angle_norm = angle_step.norm();
        if (angle_norm > 1e-12) {
            q = (q * Quaterniond(AngleAxisd(angle_norm, angle_step / angle_norm))).normalized();
        }

        Matrix<double, 15, 15> Fx = Matrix<double, 15, 15>::Identity();
        Fx.block<3, 3>(0, 3) = Matrix3d::Identity() * dt;
        Fx.block<3, 3>(3, 6) = -R_mat * skew(acc_body) * dt;
        Fx.block<3, 3>(3, 9) = -R_mat * dt;
        if (angle_norm > 1e-12) {
            Fx.block<3, 3>(6, 6) = AngleAxisd(angle_norm, angle_step / angle_norm).toRotationMatrix().transpose();
        } else {
            Fx.block<3, 3>(6, 6) = Matrix3d::Identity();
        }
        Fx.block<3, 3>(6, 12) = -Matrix3d::Identity() * dt;

        Matrix<double, 15, 15> Qi = Matrix<double, 15, 15>::Zero();
        Qi.block<3, 3>(3, 3) = std::pow(sigma_an * dt, 2) * Matrix3d::Identity();
        Qi.block<3, 3>(6, 6) = std::pow(sigma_wn * dt, 2) * Matrix3d::Identity();
        Qi.block<3, 3>(9, 9) = (sigma_aw * sigma_aw * dt) * Matrix3d::Identity();
        Qi.block<3, 3>(12, 12) = (sigma_ww * sigma_ww * dt) * Matrix3d::Identity();

        P = Fx * P * Fx.transpose() + Qi;
        P = (P + P.transpose()) * 0.5;
    }

    void update_mag(const Vector3d& mag_meas, double R_noise_scalar) {
        if (mag_meas.array().isNaN().any()) return;

        Vector3d m_ref(1.0, 0.0, 0.0);
        Matrix3d R_mat = q.toRotationMatrix();
        Vector3d m_meas_world = R_mat * mag_meas;

        double psi_ref = std::atan2(m_ref.y(), m_ref.x());
        double psi_meas = std::atan2(m_meas_world.y(), m_meas_world.x());

        double yaw_error = psi_ref - psi_meas;
        yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

        Matrix<double, 1, 15> H = Matrix<double, 1, 15>::Zero();
        H.block<1, 3>(0, 6) = R_mat.row(2);

        Matrix<double, 1, 1> R;
        R << R_noise_scalar;
        
        kalman_correct_scalar(yaw_error, H, R);
    }

    void update_marvelmind(const Vector3d& pos_marvel, const Matrix3d& R_marvel) {
        if (pos_marvel.array().isNaN().any()) return;
        Matrix<double, 3, 15> H = Matrix<double, 3, 15>::Zero();
        H.block<3, 3>(0, 0) = Matrix3d::Identity();
        kalman_correct_vector(pos_marvel - p, H, R_marvel);
    }

    void update_slam(const Vector3d& pos_slam, const Vector4d& quat_slam_vec, 
                     const Matrix3d& R_pos, const Matrix3d& R_quat) {
        if (!pos_slam.array().isNaN().any()) {
            Matrix<double, 3, 15> H = Matrix<double, 3, 15>::Zero();
            H.block<3, 3>(0, 0) = Matrix3d::Identity();
            kalman_correct_vector(pos_slam - p, H, R_pos);
        }
        if (!quat_slam_vec.array().isNaN().any()) {
            Quaterniond q_slam(quat_slam_vec(0), quat_slam_vec(1), quat_slam_vec(2), quat_slam_vec(3));
            Quaterniond q_err = q.inverse() * q_slam;
            Vector3d dtheta = 2.0 * q_err.vec();
            if (q_err.w() < 0) dtheta = -dtheta;

            Matrix<double, 3, 15> H = Matrix<double, 3, 15>::Zero();
            H.block<3, 3>(0, 6) = Matrix3d::Identity();
            kalman_correct_vector(dtheta, H, R_quat);
        }
    }

private:
    void inject_error(const Vector<double, 15>& dx) {
        p += dx.segment<3>(0);
        v += dx.segment<3>(3);
        Vector3d dtheta = dx.segment<3>(6);
        if (dtheta.norm() > 1e-12) {
            q = (q * Quaterniond(AngleAxisd(dtheta.norm(), dtheta.normalized()))).normalized();
        }
        ab += dx.segment<3>(9);
        wb += dx.segment<3>(12);
    }

    void kalman_correct_vector(const Vector3d& innov, const Matrix<double, 3, 15>& H, const Matrix3d& R) {
        Matrix3d S = H * P * H.transpose() + R;
        Matrix<double, 15, 3> K = P * H.transpose() * S.inverse();
        Vector<double, 15> dx = K * innov;

        inject_error(dx);

        Matrix<double, 15, 15> I_KH = Matrix<double, 15, 15>::Identity() - K * H;
        P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
        
        Matrix<double, 15, 15> G = Matrix<double, 15, 15>::Identity();
        G.block<3, 3>(6, 6) = Matrix3d::Identity() - skew(0.5 * dx.segment<3>(6));
        P = (G * P * G.transpose() + (G * P * G.transpose()).transpose()) * 0.5;
    }

    void kalman_correct_scalar(double innov, const Matrix<double, 1, 15>& H, const Matrix<double, 1, 1>& R) {
        Matrix<double, 1, 1> S = H * P * H.transpose() + R;
        Matrix<double, 15, 1> K = P * H.transpose() * S.inverse();
        Vector<double, 15> dx = K * innov;

        inject_error(dx);

        Matrix<double, 15, 15> I_KH = Matrix<double, 15, 15>::Identity() - K * H;
        P = I_KH * P * I_KH.transpose() + K * R * K.transpose();

        Matrix<double, 15, 15> G = Matrix<double, 15, 15>::Identity();
        G.block<3, 3>(6, 6) = Matrix3d::Identity() - skew(0.5 * dx.segment<3>(6));
        P = (G * P * G.transpose() + (G * P * G.transpose()).transpose()) * 0.5;
    }
};

PYBIND11_MODULE(eskf_observer_cpp, m) {
    py::class_<ErrorStateKF>(m, "ErrorStateKF")
        .def(py::init<double>())
        .def_readwrite("dt", &ErrorStateKF::dt)
        .def_readwrite("p", &ErrorStateKF::p)
        .def_readwrite("v", &ErrorStateKF::v)
        .def_property("q", 
            [](const ErrorStateKF &s) { return Vector4d(s.q.w(), s.q.x(), s.q.y(), s.q.z()); },
            [](ErrorStateKF &s, const Vector4d &v) { s.q = Quaterniond(v(0), v(1), v(2), v(3)); })
        .def_readwrite("ab", &ErrorStateKF::ab)
        .def_readwrite("wb", &ErrorStateKF::wb)
        .def_readwrite("P", &ErrorStateKF::P)
        .def("predict", &ErrorStateKF::predict)
        .def("update_mag", &ErrorStateKF::update_mag)
        .def("update_marvelmind", &ErrorStateKF::update_marvelmind)
        .def("update_slam", &ErrorStateKF::update_slam);
}