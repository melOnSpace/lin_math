#ifndef LIN_MATH3D
#define LIN_MATH3D

#include <stdio.h>

static const float  PI_F = { 3.141592f };
static const double PI_D = { 3.14159265358979323846 };

static const float  e_F = { 2.718281f };
static const double e_D = { 2.71828182845904523536 };

// Type Definition of various vectors, a quaternion, and a 4x4
// matrix type! All types are unions, therefore their members can
// be accessed in different ways depending on the context. A
// comment next to the union member will explain (hopefully) their
// context.
// 
// All given Vectors can be 2D, 3D, or 4D. They are defined as
// v#_t where 2 <= # <= 4. For example a 3D Vector is a v3_t; a
// 2D Vector is v2_t. Vectors are a struct of # floats.
// 
// A Quaternion is a stuct of 4 floating point numbers just as a
// 4D Vector. The only real difference is the members and name 
// qt_t. I might merge v4_t and qt_t and trust the programmer to
// remember if a struct contains a quaternion or 4D vector.
// 
// Matrices have only 2 union "context" (don't know the right word)
// which are 16 floats or a 2D array both in column row order. The
// Matrix type m4x4_t might get more "context" later if I can think
// of any useful stuff. If you're using OpenGL and need to pass a
// m4x4_t to a 4 by 4 matrix uniform then do the following!
// glUniformMatrix4fv(location, count, GL_FALSE, (GLfloat*)mat.m);
// 
// The location and count is up to you, and transpose should be
// GL_FALSE. To pass the matrix "mat" to a shader cast it's 'm'
// member as a GLfloat pointer. Of course you don't need to name
// the matrix "mat", name it whatever.
// ----------------------------------------------------------------

typedef union {
    struct { float x, y; };
    struct { float u, v; };
    struct { float r, g; };
    struct { float real, i; };
    struct { float width, height; };

    float data[2];
} v2_t;

typedef union {
    struct { float x, y, z; };
    struct { float u, v, t; };
    struct { float r, g, b; };
    struct { float real, i, j; };
    struct { float width, height, depth; };
    struct { float pitch, yaw, roll; };

    v2_t v2;
    float data[3];
} v3_t;

typedef union {
    struct { float x, y, z, w; };
    struct { float u, v, t, s; };
    struct { float r, g, b, a; };
    struct { float real, i, j, k; };
    struct { float width, height, depth, length; };
    struct { v3_t axis; float angle; };

    v2_t v2; v3_t v3;
    float data[4];
} v4_t;

typedef union {
    struct { float real, i, j, k; };
    struct { float a, b, c, d; };
    struct { float q0, q1, q2, q3; };
    struct { float w; v3_t v; };

    v4_t v4;
    float data[4];
} qt_t;

typedef union {
    struct {
        float m00, m01, m02, m03;
        float m10, m11, m12, m13;
        float m20, m21, m22, m23;
        float m30, m31, m32, m33;
    };

    float m[4][4]; // Column Major
} m4x4_t;

// 2D Vector functions! These are just the
// decorations
// ---------------------------------------

static inline v2_t v2(float x, float y);
static inline v2_t v2Init(float i);

static inline v2_t v2_add(v2_t a, v2_t b);
static inline v2_t v2_adds(v2_t v, float s);
static inline v2_t v2_sub(v2_t a, v2_t b);
static inline v2_t v2_subs(v2_t v, float s);
static inline v2_t v2_mul(v2_t a, v2_t b);
static inline v2_t v2_muls(v2_t v, float s);
static inline v2_t v2_div(v2_t a, v2_t b);
static inline v2_t v2_divs(v2_t v, float s);

static inline v2_t v2_proj(v2_t a, v2_t b);
static inline v2_t v2_norm(v2_t v);

static inline float v2_mag(v2_t v);
static inline float v2_fastmag(v2_t v);
static inline float v2_do(v2_t a, v2_t b);
static inline float v2_angle(v2_t a, v2_t b);
static inline int v2_isZero(v2_t v);

static void v2_print(v2_t v);

// 3D Vector functions! These are just the
// decorations
// ---------------------------------------

static inline v3_t v3(float x, float y);
static inline v3_t v3Init(float i);

static inline v3_t v3_add(v3_t a, v3_t b);
static inline v3_t v3_adds(v3_t v, float s);
static inline v3_t v3_sub(v3_t a, v3_t b);
static inline v3_t v3_subs(v3_t v, float s);
static inline v3_t v3_mul(v3_t a, v3_t b);
static inline v3_t v3_muls(v3_t v, float s);
static inline v3_t v3_div(v3_t a, v3_t b);
static inline v3_t v3_divs(v3_t v, float s);

static inline v3_t v3_cross(v3_t a, v3_t b);
static inline v3_t v3_proj(v3_t a, v3_t b);
static inline v3_t v3_norm(v3_t v);

static inline float v3_mag(v3_t v);
static inline float v3_fastmag(v3_t v);
static inline float v3_do(v3_t a, v3_t b);
static inline float v3_angle(v3_t a, v3_t b);
static inline int v3_isZero(v3_t v);

static void v3_print(v3_t v);

// 4D Vector functions! These are just the
// decorations
// ---------------------------------------

static inline v4_t v4(float x, float y);
static inline v4_t v4Init(float i);

static inline v4_t v4_add(v4_t a, v4_t b);
static inline v4_t v4_adds(v4_t v, float s);
static inline v4_t v4_sub(v4_t a, v4_t b);
static inline v4_t v4_subs(v4_t v, float s);
static inline v4_t v4_mul(v4_t a, v4_t b);
static inline v4_t v4_muls(v4_t v, float s);
static inline v4_t v4_div(v4_t a, v4_t b);
static inline v4_t v4_divs(v4_t v, float s);

static inline v4_t v4_cross(v4_t a, v4_t b);
static inline v4_t v4_proj(v4_t a, v4_t b);
static inline v4_t v4_norm(v4_t v);

static inline float v4_mag(v4_t v);
static inline float v4_fastmag(v4_t v);
static inline float v4_do(v4_t a, v4_t b);
static inline float v4_angle(v4_t a, v4_t b);
static inline int v4_isZero(v4_t v);

static void v4_print(v4_t v);

// Casting vectors to other vector types <3
// These will all cast one type to another. Data can
// and will often be lost durning a cast to lower
// vectors, so keep that in mind!
// -------------------------------------------------

static inline v2_t v3_to_v2();
static inline v2_t v4_to_v2();

static inline v3_t v2_to_v3();
static inline v3_t v4_to_v3();

static inline v4_t v2_to_v4();
static inline v4_t v3_to_v4();

// Various Matrix Functions! Order time! Init functions, then
// common matrices, OpenGL camera matrices, a short cut for a
// final combined OpenGL camera matrix, matrix operations, 
// vector-matrix multiplication, and printing.
// -----------------------------------------------------------

static inline m4x4_t m4(
        float m00, float m01, float m02, float m03,
        float m10, float m11, float m12, float m13,
        float m20, float m21, float m22, float m23,
        float m30, float m31, float m32, float m33);
static inline m4x4_t m4Init(float i);

static inline m4x4_t m4_identity(void);
static inline m4x4_t m4_translation(v3_t offset);
static inline m4x4_t m4_scaling(v3_t scale);

static inline m4x4_t m4_perspective(float fovy, float aratio, float near, float far);
static inline m4x4_t m4_ortho(
        float left, float right,
        float bottom, float top,
        float back, float front);

static inline m4x4_t m4_transpose(m4x4_t m);
static inline m4x4_t m4_add(m4x4_t a, m4x4_t b);
static inline m4x4_t m4_adds(m4x4_t v, float s);
static inline m4x4_t m4_mul(m4x4_t a, m4x4_t b);
static inline m4x4_t m4_muls(m4x4_t v, float s);

static inline v4_t m4_mulv2(m4x4_t m, v2_t v);
static inline v4_t m4_mulv3(m4x4_t m, v3_t v);
static inline v4_t m4_mulv4(m4x4_t m, v4_t v);

static void m4_print(m4x4_t m);

// Quaternion  Math! Ordered in init, common quaternions,
// getting given components, quaternion arithmetic,
// quaternion functions, quaternion exponents and slerp,
// quaternion booleans, and printing.
// ------------------------------------------------------



#endif
