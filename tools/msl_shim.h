// Minimal C++ stand-ins for the MSL constructs used in ofx/metal_kernels.h (test only).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstring>

#define kernel
#define device
#define constant
#define thread

struct float4 {
    float x = 0, y = 0, z = 0, w = 0;
    float4() = default;
    explicit float4(float v) : x(v), y(v), z(v), w(v) {}
    float4& operator+=(const float4& o) { x += o.x; y += o.y; z += o.z; w += o.w; return *this; }
};
inline float4 operator*(const float4& a, float s) { float4 r; r.x = a.x * s; r.y = a.y * s; r.z = a.z * s; r.w = a.w * s; return r; }

struct uint2 { unsigned x, y; };

using std::floor;
using std::fabs;
using std::max;
