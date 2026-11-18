// Small rotation toolkit (no external dependency).
// Conventions (same as the Python prototype):
//   camera axes: x right, y down, z forward;  world: y = gravity (down)
//   Q = Ry(yaw) * Rx(pitch) * Rz(roll)   (camera -> world, "YXZ" intrinsic)
#pragma once
#include <array>
#include <cmath>

namespace ts {

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
    double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
};

struct Mat3 {
    std::array<double, 9> m{1, 0, 0, 0, 1, 0, 0, 0, 1};  // row-major
    double& operator()(int r, int c) { return m[r * 3 + c]; }
    double operator()(int r, int c) const { return m[r * 3 + c]; }
    Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r(i, j) = (*this)(i, 0) * o(0, j) + (*this)(i, 1) * o(1, j) + (*this)(i, 2) * o(2, j);
        return r;
    }
    Vec3 operator*(const Vec3& v) const {
        return {(*this)(0, 0) * v.x + (*this)(0, 1) * v.y + (*this)(0, 2) * v.z,
                (*this)(1, 0) * v.x + (*this)(1, 1) * v.y + (*this)(1, 2) * v.z,
                (*this)(2, 0) * v.x + (*this)(2, 1) * v.y + (*this)(2, 2) * v.z};
    }
    Mat3 T() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r(i, j) = (*this)(j, i);
        return r;
    }
};

inline Mat3 rotX(double a) { Mat3 r; double c = std::cos(a), s = std::sin(a); r.m = {1, 0, 0, 0, c, -s, 0, s, c}; return r; }
inline Mat3 rotY(double a) { Mat3 r; double c = std::cos(a), s = std::sin(a); r.m = {c, 0, s, 0, 1, 0, -s, 0, c}; return r; }
inline Mat3 rotZ(double a) { Mat3 r; double c = std::cos(a), s = std::sin(a); r.m = {c, -s, 0, s, c, 0, 0, 0, 1}; return r; }

/// Euler (yaw, pitch, roll) -> camera-to-world rotation
inline Mat3 fromEuler(const Vec3& e) { return rotY(e.x) * rotX(e.y) * rotZ(e.z); }

/// rotation matrix -> rotation vector (axis * angle)
inline Vec3 rotvec(const Mat3& R) {
    double c = std::max(-1.0, std::min(1.0, (R(0, 0) + R(1, 1) + R(2, 2) - 1.0) * 0.5));
    double a = std::acos(c);
    Vec3 w{R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)};
    double s = std::sin(a);
    if (s < 1e-9) return w * 0.5;  // small angle
    return w * (a / (2 * s));
}

} // namespace ts
