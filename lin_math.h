#ifndef LIN_MATH3D
#define LIN_MATH3D

#include <math.h>
#include <stdio.h>


#define PI_F 3.141592f
#define E_F 2.718281f
#define EPSILON_F 1.192092896e-07f


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
    struct { float x, y; };          // Vector
    struct { float u, v; };          // Texture
    struct { float r, g; };          // Color
    struct { float real, i; };       // Imaginary
    struct { float width, height; }; // Size

    float data[2]; // Raw Data
} v2_t;

typedef union {
    struct { float x, y, z; };              // Vector
    struct { float u, v, t; };              // Texture
    struct { float r, g, b; };              // Color
    struct { float real, i, j; };           // Imaginary
    struct { float width, height, depth; }; // Size
    struct { float pitch, yaw, roll; };     // Euler Angle

    v2_t v2;       // Cast down
    float data[3]; // Raw Data
} v3_t;

typedef union {
    struct { float x, y, z, w; };                 // Vector
    struct { float u, v, t, s; };                 // Texture (4D textures?)
    struct { float r, g, b, a; };                 // Color
    struct { float real, i, j, k; };              // Quaternion (imaginary)
    struct { float width, height, depth, time; }; // 4D Sizes ¯\_(ツ)_/¯
    struct { v3_t axis; float angle; };           // Axis Angle

    v2_t v2; v3_t v3; // Cast down
    float data[4];    // Raw Data
} v4_t;

typedef union {
    struct { v4_t c0, c1, c2, c3; };
    struct {
        float x0, y0, z0, w0;
        float x1, y1, z1, w1;
        float x2, y2, z2, w2;
        float x3, y3, z3, w3;
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
static inline v2_t v2_lerp(v2_t a, v2_t b, float t);

static inline float v2_mag(v2_t v);
static inline float v2_fastmag(v2_t v);
static inline float v2_dot(v2_t a, v2_t b);
static inline float v2_angle(v2_t a, v2_t b);
static inline int v2_isZero(v2_t v);

// Prints a vector. Workds with both v2_t & v2_d
#define v2_print(v) (printf("[%f, %f]", (v).x, (v).y))
// Prints a vector. Workds with both v2_t & v2_d
#define v2_println(v) (printf("[%f, %f]\n", (v).x, (v).y))

// 3D Vector functions! These are just the
// decorations
// ---------------------------------------

static inline v3_t v3(float x, float y, float z);
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
static inline v3_t v3_lerp(v3_t a, v3_t b, float t);

static inline float v3_mag(v3_t v);
static inline float v3_fastmag(v3_t v);
static inline float v3_dot(v3_t a, v3_t b);
static inline float v3_angle(v3_t a, v3_t b);
static inline int v3_isZero(v3_t v);

// Prints a vector. Workds with both v3_t & v3_d
#define v3_print(v) (printf("[%f, %f, %f]", (v).x, (v).y, (v).z))
// Prints a vector. Workds with both v3_t & v3_d
#define v3_println(v) (printf("[%f, %f, %f]\n", (v).x, (v).y, (v).z))

// 4D Vector functions! These are just the
// decorations
// ---------------------------------------

static inline v4_t v4(float x, float y, float z, float w);
static inline v4_t v4Init(float i);

static inline v4_t v4_add(v4_t a, v4_t b);
static inline v4_t v4_adds(v4_t v, float s);
static inline v4_t v4_sub(v4_t a, v4_t b);
static inline v4_t v4_subs(v4_t v, float s);
static inline v4_t v4_mul(v4_t a, v4_t b);
static inline v4_t v4_muls(v4_t v, float s);
static inline v4_t v4_div(v4_t a, v4_t b);
static inline v4_t v4_divs(v4_t v, float s);

static inline v4_t v4_proj(v4_t a, v4_t b);
static inline v4_t v4_norm(v4_t v);
static inline v4_t v4_normAxis(v4_t v);
static inline v4_t v4_lerp(v4_t a, v4_t b, float t);

static inline float v4_mag(v4_t v);
static inline float v4_fastmag(v4_t v);
static inline float v4_dot(v4_t a, v4_t b);
static inline float v4_angle(v4_t a, v4_t b);
static inline int v4_isZero(v4_t v);

// Prints a vector. Workds with both v4_t & v4_d
#define v4_print(v) (printf("[%f, %f, %f, %f]", (v).x, (v).y, (v).z, (v).w))
// Prints a vector. Workds with both v4_t & v4_d 
#define v4_println(v) (printf("[%f, %f, %f, %f]\n", (v).x, (v).y, (v).z, (v).w))

// Casting vectors to other vector types <3
// These will all cast one type to another. Data can
// and will often be lost durning a cast to lower
// vectors, so keep that in mind!
// -------------------------------------------------

static inline v2_t v3_to_v2(v3_t v);
static inline v2_t v4_to_v2(v4_t v);

static inline v3_t v2_to_v3(v2_t v);
static inline v3_t v4_to_v3(v4_t v);

static inline v4_t v2_to_v4(v2_t v);
static inline v4_t v3_to_v4(v3_t v);

// Various Matrix Functions!
// -------------------------

static inline m4x4_t m4v(v4_t c0, v4_t c1, v4_t c2, v4_t c3);
static inline m4x4_t m4(
        float x0, float x1, float x2, float x3,
        float y0, float y1, float y2, float y3,
        float z0, float z1, float z2, float z3,
        float w0, float w1, float w2, float w3);
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
static inline m4x4_t m4_adds(m4x4_t m, float s);
static inline m4x4_t m4_sub(m4x4_t a, m4x4_t b);
static inline m4x4_t m4_subs(m4x4_t m, float s);
static inline m4x4_t m4_mul(m4x4_t a, m4x4_t b);
static inline m4x4_t m4_muls(m4x4_t m, float s);

static inline v4_t m4_mulv2(m4x4_t m, v2_t v);
static inline v4_t m4_mulv3(m4x4_t m, v3_t v);
static inline v4_t m4_mulv4(m4x4_t m, v4_t v);

static inline v4_t m4_v2mul(v2_t v, m4x4_t m);
static inline v4_t m4_v3mul(v3_t v, m4x4_t m);
static inline v4_t m4_v4mul(v4_t v, m4x4_t m);


#define MATRIX_PRINT_STR \
"/ %f %f %f %f \\\
| %f %f %f %f |\
| %f %f %f %f |\
\\ %f %f %f %f /"

// Prints a matrix. Workds with both m4x4_t & m4x4_d
#define m4_println(m) (printf(MATRIX_PRINT_STR, \
            (m).x0, (m).x1, (m).x2, (m).x3,\
            (m).y0, (m).y1, (m).y2, (m).y3,\
            (m).z0, (m).z1, (m).z2, (m).z3,\
            (m).w0, (m).w1, (m).w2, (m).w3)


#define LIN_MATH3D_IMPLEMENTATION
#ifdef LIN_MATH3D_IMPLEMENTATION

// 2D Vectors Implementaions
// -------------------------

static inline v2_t v2(float x, float y) {
    return (v2_t){ .x=x, .y=y };
}

static inline v2_t v2Init(float i) {
    return (v2_t){ .x=i, .y=i };
}


static inline v2_t v2_add(v2_t a, v2_t b) {
    return (v2_t){
        .x=(a.x + b.x),
        .y=(a.y + b.y),
    };
}

static inline v2_t v2_adds(v2_t v, float s) {
    return (v2_t){
        .x=(v.x + s),
        .y=(v.y + s),
    };
}

static inline v2_t v2_sub(v2_t a, v2_t b) {
    return (v2_t){
        .x=(a.x - b.x),
        .y=(a.y - b.y),
    };
}

static inline v2_t v2_subs(v2_t v, float s) {
    return (v2_t){
        .x=(v.x - s),
        .y=(v.y - s),
    };
}

static inline v2_t v2_mul(v2_t a, v2_t b) {
    return (v2_t){
        .x=(a.x * b.x),
        .y=(a.y * b.y),
    };
}

static inline v2_t v2_muls(v2_t v, float s) {
    return (v2_t){
        .x=(v.x * s),
        .y=(v.y * s),
    };
}

static inline v2_t v2_div(v2_t a, v2_t b) {
    return (v2_t){
        .x=(a.x / b.x),
        .y=(a.y / b.y),
    };
}

static inline v2_t v2_divs(v2_t v, float s) {
    return (v2_t){
        .x=(v.x / s),
        .y=(v.y / s),
    };
}


static inline v2_t v2_proj(v2_t a, v2_t b) {
    float s = v2_dot(a, b) / v2_dot(b, b);
    return v2_muls(b, s);
}

static inline v2_t v2_norm(v2_t v) {
    return v2_divs(v, v2_mag(v));
}

static inline v2_t v2_lerp(v2_t a, v2_t b, float t) {
    return v2_add(v2_muls(a, 1.0f - t), v2_muls(b, t));
}


static inline float v2_mag(v2_t v) {
    return sqrtf(v.x * v.x + v.y * v.y);
}

static inline float v2_fastmag(v2_t v) {
    return v.x * v.x + v.y * v.y;
}

static inline float v2_dot(v2_t a, v2_t b) {
    return a.x * b.x + a.y * b.y;
}

static inline float v2_angle(v2_t a, v2_t b) {
    return acosf(v2_dot(a, b) / (v2_mag(a) * v2_mag(b)));
}

static inline int v2_isZero(v2_t v) {
    return v2_fastmag(v) < EPSILON_F;
}


// 3D Vectors Implementaions
// -------------------------

static inline v3_t v3(float x, float y, float z) {
    return (v3_t){ .x=x, .y=y, .z=z };
}

static inline v3_t v3Init(float i) {
    return (v3_t){ .x=i, .y=i, .z=i };
}


static inline v3_t v3_add(v3_t a, v3_t b) {
    return (v3_t){
        .x=(a.x + b.x),
        .y=(a.y + b.y),
        .z=(a.z + b.z),
    };
}

static inline v3_t v3_adds(v3_t v, float s) {
    return (v3_t){
        .x=(v.x + s),
        .y=(v.y + s),
        .z=(v.z + s),
    };
}

static inline v3_t v3_sub(v3_t a, v3_t b) {
    return (v3_t){
        .x=(a.x - b.x),
        .y=(a.y - b.y),
        .z=(a.z - b.z),
    };
}

static inline v3_t v3_subs(v3_t v, float s) {
    return (v3_t){
        .x=(v.x - s),
        .y=(v.y - s),
        .z=(v.z - s),
    };
}

static inline v3_t v3_mul(v3_t a, v3_t b) {
    return (v3_t){
        .x=(a.x * b.x),
        .y=(a.y * b.y),
        .z=(a.z * b.z),
    };
}

static inline v3_t v3_muls(v3_t v, float s) {
    return (v3_t){
        .x=(v.x * s),
        .y=(v.y * s),
        .z=(v.z * s),
    };
}

static inline v3_t v3_div(v3_t a, v3_t b) {
    return (v3_t){
        .x=(a.x / b.x),
        .y=(a.y / b.y),
        .z=(a.z / b.z),
    };
}

static inline v3_t v3_divs(v3_t v, float s) {
    return (v3_t){
        .x=(v.x / s),
        .y=(v.y / s),
        .z=(v.z / s),
    };
}

static inline v3_t v3_cross(v3_t a, v3_t b) {
    return (v3_t){
        .x = (a.y * b.z) - (a.z * b.y),
        .y = (a.z * b.x) - (a.x * b.z),
        .z = (a.x * b.y) - (a.y * b.x),
    };
}

static inline v3_t v3_proj(v3_t a, v3_t b) {
    float s = v3_dot(a, b) / v3_dot(b, b);
    return v3_muls(b, s);
}

static inline v3_t v3_norm(v3_t v) {
    return v3_divs(v, v3_mag(v));
}

static inline v3_t v3_lerp(v3_t a, v3_t b, float t) {
    return v3_add(v3_muls(a, 1.0f - t), v3_muls(b, t));
}


static inline float v3_mag(v3_t v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static inline float v3_fastmag(v3_t v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

static inline float v3_dot(v3_t a, v3_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline float v3_angle(v3_t a, v3_t b) {
    return acosf(v3_dot(a, b) / (v3_mag(a) * v3_mag(b)));
}

static inline int v3_isZero(v3_t v) {
    return v3_fastmag(v) < EPSILON_F;
}


// 4D Vectors Implementaions
// -------------------------

static inline v4_t v4(float x, float y, float z, float w) {
    return (v4_t){ .x=x, .y=y, .z=z, .w=w };
}

static inline v4_t v4Init(float i) {
    return (v4_t){ .x=i, .y=i, .z=i, .w=i };
}


static inline v4_t v4_add(v4_t a, v4_t b) {
    return (v4_t){
        .x=(a.x + b.x),
        .y=(a.y + b.y),
        .z=(a.z + b.z),
        .w=(a.w + b.w),
    };
}

static inline v4_t v4_adds(v4_t v, float s) {
    return (v4_t){
        .x=(v.x + s),
        .y=(v.y + s),
        .z=(v.z + s),
        .w=(v.w + s),
    };
}

static inline v4_t v4_sub(v4_t a, v4_t b) {
    return (v4_t){
        .x=(a.x - b.x),
        .y=(a.y - b.y),
        .z=(a.z - b.z),
        .w=(a.w - b.w),
    };
}

static inline v4_t v4_subs(v4_t v, float s) {
    return (v4_t){
        .x=(v.x - s),
        .y=(v.y - s),
        .z=(v.z - s),
        .w=(v.w - s),
    };
}

static inline v4_t v4_mul(v4_t a, v4_t b) {
    return (v4_t){
        .x=(a.x * b.x),
        .y=(a.y * b.y),
        .z=(a.z * b.z),
        .w=(a.w * b.w),
    };
}

static inline v4_t v4_muls(v4_t v, float s) {
    return (v4_t){
        .x=(v.x * s),
        .y=(v.y * s),
        .z=(v.z * s),
        .w=(v.w * s),
    };
}

static inline v4_t v4_div(v4_t a, v4_t b) {
    return (v4_t){
        .x=(a.x / b.x),
        .y=(a.y / b.y),
        .z=(a.z / b.z),
        .w=(a.w / b.w),
    };
}

static inline v4_t v4_divs(v4_t v, float s) {
    return (v4_t){
        .x=(v.x / s),
        .y=(v.y / s),
        .z=(v.z / s),
        .w=(v.w / s),
    };
}


static inline v4_t v4_proj(v4_t a, v4_t b) {
    float s = v4_dot(a, b) / v4_dot(b, b);
    return v4_muls(b, s);
}

static inline v4_t v4_norm(v4_t v) {
    return v4_divs(v, v4_mag(v));
}

static inline v4_t v4_normAxis(v4_t v) {
    return (v4_t){
        .angle = v.angle,
        .axis = v3_norm(v.axis),
    };
}

static inline v4_t v4_lerp(v4_t a, v4_t b, float t) {
    return v4_add(v4_muls(a, 1.0f - t), v4_muls(b, t));
}


static inline float v4_mag(v4_t v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
}

static inline float v4_fastmag(v4_t v) {
    return v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w;
}

static inline float v4_dot(v4_t a, v4_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static inline float v4_angle(v4_t a, v4_t b) {
    return acosf(v4_dot(a, b) / (v4_mag(a) * v4_mag(b)));
}

static inline int v4_isZero(v4_t v) {
    return v4_fastmag(v) < EPSILON_F;
}



// Vector Casting Implementaions
// -----------------------------

static inline v2_t v3_to_v2(v3_t v) {
    return v.v2;
}

static inline v2_t v4_to_v2(v4_t v) {
    return v.v2;
}


static inline v3_t v2_to_v3(v2_t v) {
    return (v3_t){ .x=v.x, .y=v.y, .z=0.0f };
}

static inline v3_t v4_to_v3(v4_t v) {
    return v.v3;
}


static inline v4_t v2_to_v4(v2_t v) {
    return (v4_t){ .x=v.x, .y=v.y, .z=0.0f, .w=0.0f };
}

static inline v4_t v3_to_v4(v3_t v) {
    return (v4_t){ .x=v.x, .y=v.y, .z=v.z, .w=0.0f };
}


// Matrix Functions and the math!
// ------------------------------


static inline m4x4_t m4v(v4_t c0, v4_t c1, v4_t c2, v4_t c3) {
    return (m4x4_t){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}

static inline m4x4_t m4(
        float x0, float x1, float x2, float x3,
        float y0, float y1, float y2, float y3,
        float z0, float z1, float z2, float z3,
        float w0, float w1, float w2, float w3) {
    return (m4x4_t){
        .x0=x0, .y0=y0, .z0=z0, .w0=w0,
        .x1=x1, .y1=y1, .z1=z1, .w1=w1,
        .x2=x2, .y2=y2, .z2=z2, .w2=w2,
        .x3=x3, .y3=y3, .z3=z3, .w3=w3,
    };
}

static inline m4x4_t m4Init(float i) {
    return (m4x4_t){
        .x0=i, .y0=i, .z0=i, .w0=i,
        .x1=i, .y1=i, .z1=i, .w1=i,
        .x2=i, .y2=i, .z2=i, .w2=i,
        .x3=i, .y3=i, .z3=i, .w3=i,
    };
}

static inline m4x4_t m4_identity(void) {
    return (m4x4_t){
        .x0=1.0f, .y0=0.0f, .z0=0.0f, .w0=0.0f,
        .x1=0.0f, .y1=1.0f, .z1=0.0f, .w1=0.0f,
        .x2=0.0f, .y2=0.0f, .z2=1.0f, .w2=0.0f,
        .x3=0.0f, .y3=0.0f, .z3=0.0f, .w3=1.0f,
    };
}

static inline m4x4_t m4_translation(v3_t offset) {
    return (m4x4_t){
        .x0=1.0f,     .y0=0.0f,     .z0=0.0f,     .w0=0.0f,
        .x1=0.0f,     .y1=1.0f,     .z1=0.0f,     .w1=0.0f,
        .x2=0.0f,     .y2=0.0f,     .z2=1.0f,     .w2=0.0f,
        .x3=offset.x, .y3=offset.y, .z3=offset.z, .w3=1.0f,
    };
}

static inline m4x4_t m4_scaling(v3_t scale) {
    return (m4x4_t){
        .x0=scale.x, .y0=0.0f,    .z0=0.0f,    .w0=0.0f,
        .x1=0.0f,    .y1=scale.y, .z1=0.0f,    .w1=0.0f,
        .x2=0.0f,    .y2=0.0f,    .z2=scale.z, .w2=0.0f,
        .x3=0.0f,    .y3=0.0f,    .z3=0.0f,    .w3=1.0f,
    };
}


static inline m4x4_t m4_perspective(float fovy, float aratio, float near, float far) {
    float f = 1.0f / tanf(fovy / 2.0f);

    m4x4_t result = m4_scaling((v3_t){
        .x = f / aratio,
        .y = f,
        .z = (far + near) / (near - far),
    });

    result.z3 = (2.0f * far * near) / (near - far);
    result.w2 = -1.0f;

    return result;
}

static inline m4x4_t m4_ortho(
        float left, float right,
        float bottom, float top,
        float back, float front) {

    m4x4_t result = m4_scaling((v3_t){
            .x = 2.0f / (right - left),
            .y = 2.0f / (top - bottom),
            .z = 2.0f / (back - front),
    });

    result.x3 = -(right + left) / (right - left);
    result.y3 = -(top + bottom) / (top - bottom);
    result.z3 = -(back + front) / (back - front);

    return result;
}


static inline m4x4_t m4_transpose(m4x4_t m) {
    return (m4x4_t){
        .x0=m.x0, .y0=m.x1, .z0=m.x2, .w0=m.x3,
        .x1=m.y0, .y1=m.y1, .z1=m.y2, .w1=m.y3,
        .x2=m.z0, .y2=m.z1, .z2=m.z2, .w2=m.z3,
        .x3=m.w0, .y3=m.w1, .z3=m.w2, .w3=m.w3,
    };
}

static inline m4x4_t m4_add(m4x4_t a, m4x4_t b) {
    return (m4x4_t){
        .x0 = a.x0 + b.x0, .x1 = a.x1 + b.x1, .x2 = a.x2 + b.x2, .x3 = a.x3 + b.x3,
        .y0 = a.y0 + b.y0, .y1 = a.y1 + b.y1, .y2 = a.y2 + b.y2, .y3 = a.y3 + b.y3,
        .z0 = a.z0 + b.z0, .z1 = a.z1 + b.z1, .z2 = a.z2 + b.z2, .z3 = a.z3 + b.z3,
        .w0 = a.w0 + b.w0, .w1 = a.w1 + b.w1, .w2 = a.w2 + b.w2, .w3 = a.w3 + b.w3,
    };
}

static inline m4x4_t m4_adds(m4x4_t m, float s) {
    return (m4x4_t){
        .x0 = m.x0 + s, .x1 = m.x1 + s, .x2 = m.x2 + s, .x3 = m.x3 + s,
        .y0 = m.y0 + s, .y1 = m.y1 + s, .y2 = m.y2 + s, .y3 = m.y3 + s,
        .z0 = m.z0 + s, .z1 = m.z1 + s, .z2 = m.z2 + s, .z3 = m.z3 + s,
        .w0 = m.w0 + s, .w1 = m.w1 + s, .w2 = m.w2 + s, .w3 = m.w3 + s,
    };
}

static inline m4x4_t m4_sub(m4x4_t a, m4x4_t b) {
    return (m4x4_t){
        .x0 = a.x0 - b.x0, .x1 = a.x1 - b.x1, .x2 = a.x2 - b.x2, .x3 = a.x3 - b.x3,
        .y0 = a.y0 - b.y0, .y1 = a.y1 - b.y1, .y2 = a.y2 - b.y2, .y3 = a.y3 - b.y3,
        .z0 = a.z0 - b.z0, .z1 = a.z1 - b.z1, .z2 = a.z2 - b.z2, .z3 = a.z3 - b.z3,
        .w0 = a.w0 - b.w0, .w1 = a.w1 - b.w1, .w2 = a.w2 - b.w2, .w3 = a.w3 - b.w3,
    };
}

static inline m4x4_t m4_subs(m4x4_t m, float s) {
    return (m4x4_t){
        .x0 = m.x0 - s, .x1 = m.x1 - s, .x2 = m.x2 - s, .x3 = m.x3 - s,
        .y0 = m.y0 - s, .y1 = m.y1 - s, .y2 = m.y2 - s, .y3 = m.y3 - s,
        .z0 = m.z0 - s, .z1 = m.z1 - s, .z2 = m.z2 - s, .z3 = m.z3 - s,
        .w0 = m.w0 - s, .w1 = m.w1 - s, .w2 = m.w2 - s, .w3 = m.w3 - s,
    };
}

static inline m4x4_t m4_mul(m4x4_t a, m4x4_t b) {
    v4_t c0 = {
        .x = a.x0*b.x0 + a.x1*b.y0 + a.x2*b.z0 + a.x3*b.w0,
        .y = a.y0*b.x0 + a.y1*b.y0 + a.y2*b.z0 + a.y3*b.w0,
        .z = a.z0*b.x0 + a.z1*b.y0 + a.z2*b.z0 + a.z3*b.w0,
        .w = a.w0*b.x0 + a.w1*b.y0 + a.w2*b.z0 + a.w3*b.w0,
	};

    v4_t c1 = {
        .x = a.x0*b.x1 + a.x1*b.y1 + a.x2*b.z1 + a.x3*b.w1,
        .y = a.y0*b.x1 + a.y1*b.y1 + a.y2*b.z1 + a.y3*b.w1,
        .z = a.z0*b.x1 + a.z1*b.y1 + a.z2*b.z1 + a.z3*b.w1,
        .w = a.w0*b.x1 + a.w1*b.y1 + a.w2*b.z1 + a.w3*b.w1,
	};

    v4_t c2 = {
        .x = a.x0*b.x2 + a.x1*b.y2 + a.x2*b.z2 + a.x3*b.w2,
        .y = a.y0*b.x2 + a.y1*b.y2 + a.y2*b.z2 + a.y3*b.w2,
        .z = a.z0*b.x2 + a.z1*b.y2 + a.z2*b.z2 + a.z3*b.w2,
        .w = a.w0*b.x2 + a.w1*b.y2 + a.w2*b.z2 + a.w3*b.w2,
	};

    v4_t c3 = {
        .x = a.x0*b.x3 + a.x1*b.y3 + a.x2*b.z3 + a.x3* b.w3,
        .y = a.y0*b.x3 + a.y1*b.y3 + a.y2*b.z3 + a.y3* b.w3,
        .z = a.z0*b.x3 + a.z1*b.y3 + a.z2*b.z3 + a.z3* b.w3,
        .w = a.w0*b.x3 + a.w1*b.y3 + a.w2*b.z3 + a.w3* b.w3,
	};

    return (m4x4_t){ .c0=c0, .c1=c1, .c2=c2, .c3=c3 };
}

static inline m4x4_t m4_muls(m4x4_t m, float s) {
    return (m4x4_t){
        .x0=m.x0 * s, .y0=m.y0 * s, .z0=m.z0 * s, .w0=m.w0 * s,
        .x1=m.x1 * s, .y1=m.y1 * s, .z1=m.z1 * s, .w1=m.w1 * s,
        .x2=m.x2 * s, .y2=m.y2 * s, .z2=m.z2 * s, .w2=m.w2 * s,
        .x3=m.x3 * s, .y3=m.y3 * s, .z3=m.z3 * s, .w3=m.w3 * s,
    };
}


static inline v4_t m4_mulv2(m4x4_t m, v2_t v) {
    return (v4_t){
        .x = v.x * m.x0 + v.y * m.x1,
        .y = v.x * m.y0 + v.y * m.y1,
        .z = v.x * m.z0 + v.y * m.z1,
        .w = v.x * m.w0 + v.y * m.w1,
    };
}

static inline v4_t m4_mulv3(m4x4_t m, v3_t v) {
    return (v4_t){
        .x = v.x * m.x0 + v.y * m.x1 + v.z * m.x2,
        .y = v.x * m.y0 + v.y * m.y1 + v.z * m.y2,
        .z = v.x * m.z0 + v.y * m.z1 + v.z * m.z2,
        .w = v.x * m.w0 + v.y * m.w1 + v.z * m.w2,
    };
}

static inline v4_t m4_mulv4(m4x4_t m, v4_t v) {
    return (v4_t){
        .x = v.x * m.x0 + v.y * m.x1 + v.z * m.x2 + v.w * m.x3,
        .y = v.x * m.y0 + v.y * m.y1 + v.z * m.y2 + v.w * m.y3,
        .z = v.x * m.z0 + v.y * m.z1 + v.z * m.z2 + v.w * m.z3,
        .w = v.x * m.w0 + v.y * m.w1 + v.z * m.w2 + v.w * m.w3,
    };
}

static inline v4_t m4_v2mul(v2_t v, m4x4_t m) {
	return (v4_t){
        .x = v.x * m.x0 + v.y * m.y0,
        .y = v.x * m.x1 + v.y * m.y1,
        .z = v.x * m.x2 + v.y * m.y2,
        .w = v.x * m.x3 + v.y * m.y3,
	};
}

static inline v4_t m4_v3mul(v3_t v, m4x4_t m) {
	return (v4_t){
        .x = v.x * m.x0 + v.y * m.y0 + v.z * m.z0,
        .y = v.x * m.x1 + v.y * m.y1 + v.z * m.z1,
        .z = v.x * m.x2 + v.y * m.y2 + v.z * m.z2,
        .w = v.x * m.x3 + v.y * m.y3 + v.z * m.z3,
	};
}

static inline v4_t m4_v4mul(v4_t v, m4x4_t m) {
	return (v4_t){
        .x = v.x * m.x0 + v.y * m.y0 + v.z * m.z0 + v.w * m.w0,
        .y = v.x * m.x1 + v.y * m.y1 + v.z * m.z1 + v.w * m.w1,
        .z = v.x * m.x2 + v.y * m.y2 + v.z * m.z2 + v.w * m.w2,
        .w = v.x * m.x3 + v.y * m.y3 + v.z * m.z3 + v.w * m.w3,
	};
}


#endif

#endif


#ifdef LIN_DOUBLE

#define E_D 2.71828182845904523536
#define PI_D 3.14159265358979323846
#define EPSILON_D 2.2204460492503131e-016


typedef union {
    struct { double x, y; };
    struct { double u, v; };
    struct { double r, g; };
    struct { double real, i; };
    struct { double width, height; };

    double data[2];
} v2_d;

typedef union {
    struct { double x, y, z; };
    struct { double u, v, t; };
    struct { double r, g, b; };
    struct { double real, i, j; };
    struct { double width, height, depth; };
    struct { double pitch, yaw, roll; };

    v2_d v2;
    double data[3];
} v3_d;

typedef union {
    struct { double x, y, z, w; };
    struct { double u, v, t, s; };
    struct { double r, g, b, a; };
    struct { double real, i, j, k; };
    struct { double width, height, depth, length; };
    struct { v3_d axis; double angle; };

    v2_d v2; v3_d v3;
    double data[4];
} v4_d;

typedef union {
    struct { v4_d c0, c1, c2, c3; };
    struct {
        double x0, y0, z0, w0;
        double x1, y1, z1, w1;
        double x2, y2, z2, w2;
        double x3, y3, z3, w3;
    };

    double data[4][4]; // Column Major
} m4x4_d;

// 2D Vector functions! These are just the
// decorations
// ---------------------------------------

static inline v2_d v2d(double x, double y);
static inline v2_d v2dInit(double i);

static inline v2_d v2d_add(v2_d a, v2_d b);
static inline v2_d v2d_adds(v2_d v, double s);
static inline v2_d v2d_sub(v2_d a, v2_d b);
static inline v2_d v2d_subs(v2_d v, double s);
static inline v2_d v2d_mul(v2_d a, v2_d b);
static inline v2_d v2d_muls(v2_d v, double s);
static inline v2_d v2d_div(v2_d a, v2_d b);
static inline v2_d v2d_divs(v2_d v, double s);

static inline v2_d v2d_proj(v2_d a, v2_d b);
static inline v2_d v2d_norm(v2_d v);
static inline v2_d v2d_lerp(v2_d a, v2_d b, double t);

static inline double v2d_mag(v2_d v);
static inline double v2d_fastmag(v2_d v);
static inline double v2d_dot(v2_d a, v2_d b);
static inline double v2d_angle(v2_d a, v2_d b);
static inline int v2d_isZero(v2_d v);

int v2d_print(v2_d v);
int v2d_println(v2_d v);

// 3D Vector functions! These are just the
// decorations
// ---------------------------------------

static inline v3_d v3d(double x, double y, double z);
static inline v3_d v3dInit(double i);

static inline v3_d v3d_add(v3_d a, v3_d b);
static inline v3_d v3d_adds(v3_d v, double s);
static inline v3_d v3d_sub(v3_d a, v3_d b);
static inline v3_d v3d_subs(v3_d v, double s);
static inline v3_d v3d_mul(v3_d a, v3_d b);
static inline v3_d v3d_muls(v3_d v, double s);
static inline v3_d v3d_div(v3_d a, v3_d b);
static inline v3_d v3d_divs(v3_d v, double s);

static inline v3_d v3d_cross(v3_d a, v3_d b);
static inline v3_d v3d_proj(v3_d a, v3_d b);
static inline v3_d v3d_norm(v3_d v);
static inline v3_d v3d_lerp(v3_d a, v3_d b, double t);

static inline double v3d_mag(v3_d v);
static inline double v3d_fastmag(v3_d v);
static inline double v3d_dot(v3_d a, v3_d b);
static inline double v3d_angle(v3_d a, v3_d b);
static inline int v3d_isZero(v3_d v);

int v3d_print(v3_d v);
int v3d_println(v3_d v);

// 4D Vector functions! These are just the
// decorations
// ---------------------------------------

static inline v4_d v4d(double x, double y, double z, double w);
static inline v4_d v4dInit(double i);

static inline v4_d v4d_add(v4_d a, v4_d b);
static inline v4_d v4d_adds(v4_d v, double s);
static inline v4_d v4d_sub(v4_d a, v4_d b);
static inline v4_d v4d_subs(v4_d v, double s);
static inline v4_d v4d_mul(v4_d a, v4_d b);
static inline v4_d v4d_muls(v4_d v, double s);
static inline v4_d v4d_div(v4_d a, v4_d b);
static inline v4_d v4d_divs(v4_d v, double s);

static inline v4_d v4d_proj(v4_d a, v4_d b);
static inline v4_d v4d_norm(v4_d v);
static inline v4_d v4d_normAxis(v4_d v);
static inline v4_d v4d_lerp(v4_d a, v4_d b, double t);

static inline double v4d_mag(v4_d v);
static inline double v4d_fastmag(v4_d v);
static inline double v4d_dot(v4_d a, v4_d b);
static inline double v4d_angle(v4_d a, v4_d b);
static inline int v4d_isZero(v4_d v);

int v4d_print(v4_d v);
int v4d_println(v4_d v);

// Casting vectors to other vector types <3
// These will all cast one type to another. Data can
// and will often be lost durning a cast to lower
// vectors, so keep that in mind!
// -------------------------------------------------

static inline v2_d v3d_to_v2d(v3_d v);
static inline v2_d v4d_to_v2d(v4_d v);

static inline v3_d v2d_to_v3d(v2_d v);
static inline v3_d v4d_to_v3d(v4_d v);

static inline v4_d v2d_to_v4d(v2_d v);
static inline v4_d v3d_to_v4d(v3_d v);

// Various Matrix Functions! Order time! Init functions, then
// common matrices, OpenGL camera matrices, a short cut for a
// final combined OpenGL camera matrix, matrix operations, 
// vector-matrix multiplication, and printing.
// -----------------------------------------------------------

static inline m4x4_d m4dv(v4_d c0, v4_d c1, v4_d c2, v4_d c3);
static inline m4x4_d m4d(
        double x0, double x1, double x2, double x3,
        double y0, double y1, double y2, double y3,
        double z0, double z1, double z2, double z3,
        double w0, double w1, double w2, double w3);
static inline m4x4_d m4dInit(double i);

static inline m4x4_d m4d_identity(void);
static inline m4x4_d m4d_translation(v3_d offset);
static inline m4x4_d m4d_scaling(v3_d scale);

static inline m4x4_d m4d_perspective(double fovy, double aratio, double near, double far);
static inline m4x4_d m4d_ortho(
        double left, double right,
        double bottom, double top,
        double back, double front);

static inline m4x4_d m4d_transpose(m4x4_d m);
static inline m4x4_d m4d_add(m4x4_d a, m4x4_d b);
static inline m4x4_d m4d_adds(m4x4_d m, double s);
static inline m4x4_d m4d_sub(m4x4_d a, m4x4_d b);
static inline m4x4_d m4d_subs(m4x4_d m, double s);
static inline m4x4_d m4d_mul(m4x4_d a, m4x4_d b);
static inline m4x4_d m4d_muls(m4x4_d m, double s);

static inline v4_d m4d_mulv2(m4x4_d m, v2_d v);
static inline v4_d m4d_mulv3(m4x4_d m, v3_d v);
static inline v4_d m4d_mulv4(m4x4_d m, v4_d v);

static inline v4_d m4d_v2mul(v2_d v, m4x4_d m);
static inline v4_d m4d_v3mul(v3_d v, m4x4_d m);
static inline v4_d m4d_v4mul(v4_d v, m4x4_d m);

int m4d_print(m4x4_d m);
int m4d_println(m4x4_d m);

#ifdef LIN_DOUBLE_IMPLEMENTATION


// 2D Vectors Implementaions
// -------------------------

static inline v2_d v2d(double x, double y) {
    return (v2_d){ .x=x, .y=y };
}

static inline v2_d v2dInit(double i) {
    return (v2_d){ .x=i, .y=i };
}


static inline v2_d v2d_add(v2_d a, v2_d b) {
    return (v2_d){
        .x=(a.x + b.x),
        .y=(a.y + b.y),
    };
}

static inline v2_d v2d_adds(v2_d v, double s) {
    return (v2_d){
        .x=(v.x + s),
        .y=(v.y + s),
    };
}

static inline v2_d v2d_sub(v2_d a, v2_d b) {
    return (v2_d){
        .x=(a.x - b.x),
        .y=(a.y - b.y),
    };
}

static inline v2_d v2d_subs(v2_d v, double s) {
    return (v2_d){
        .x=(v.x - s),
        .y=(v.y - s),
    };
}

static inline v2_d v2d_mul(v2_d a, v2_d b) {
    return (v2_d){
        .x=(a.x * b.x),
        .y=(a.y * b.y),
    };
}

static inline v2_d v2d_muls(v2_d v, double s) {
    return (v2_d){
        .x=(v.x * s),
        .y=(v.y * s),
    };
}

static inline v2_d v2d_div(v2_d a, v2_d b) {
    return (v2_d){
        .x=(a.x / b.x),
        .y=(a.y / b.y),
    };
}

static inline v2_d v2d_divs(v2_d v, double s) {
    return (v2_d){
        .x=(v.x / s),
        .y=(v.y / s),
    };
}


static inline v2_d v2d_proj(v2_d a, v2_d b) {
    double s = v2d_dot(a, b) / v2d_dot(b, b);
    return v2d_muls(b, s);
}

static inline v2_d v2d_norm(v2_d v) {
    return v2d_divs(v, v2d_mag(v));
}

static inline v2_d v2d_lerp(v2_d a, v2_d b, double t) {
    return v2d_add(v2d_muls(a, 1.0 - t), v2d_muls(b, t));
}


static inline double v2d_mag(v2_d v) {
    return sqrt(v.x * v.x + v.y * v.y);
}

static inline double v2d_fastmag(v2_d v) {
    return v.x * v.x + v.y * v.y;
}

static inline double v2d_dot(v2_d a, v2_d b) {
    return a.x * b.x + a.y * b.y;
}

static inline double v2d_angle(v2_d a, v2_d b) {
    return acos(v2d_dot(a, b) / (v2d_mag(a) * v2d_mag(b)));
}

static inline int v2d_isZero(v2_d v) {
    return v2d_fastmag(v) < EPSILON_D;
}


// 3D Vectors Implementaions
// -------------------------

static inline v3_d v3d(double x, double y, double z) {
    return (v3_d){ .x=x, .y=y, .z=z };
}

static inline v3_d v3Init(double i) {
    return (v3_d){ .x=i, .y=i, .z=i };
}


static inline v3_d v3d_add(v3_d a, v3_d b) {
    return (v3_d){
        .x=(a.x + b.x),
        .y=(a.y + b.y),
        .z=(a.z + b.z),
    };
}

static inline v3_d v3d_adds(v3_d v, double s) {
    return (v3_d){
        .x=(v.x + s),
        .y=(v.y + s),
        .z=(v.z + s),
    };
}

static inline v3_d v3d_sub(v3_d a, v3_d b) {
    return (v3_d){
        .x=(a.x - b.x),
        .y=(a.y - b.y),
        .z=(a.z - b.z),
    };
}

static inline v3_d v3d_subs(v3_d v, double s) {
    return (v3_d){
        .x=(v.x - s),
        .y=(v.y - s),
        .z=(v.z - s),
    };
}

static inline v3_d v3d_mul(v3_d a, v3_d b) {
    return (v3_d){
        .x=(a.x * b.x),
        .y=(a.y * b.y),
        .z=(a.z * b.z),
    };
}

static inline v3_d v3d_muls(v3_d v, double s) {
    return (v3_d){
        .x=(v.x * s),
        .y=(v.y * s),
        .z=(v.z * s),
    };
}

static inline v3_d v3d_div(v3_d a, v3_d b) {
    return (v3_d){
        .x=(a.x / b.x),
        .y=(a.y / b.y),
        .z=(a.z / b.z),
    };
}

static inline v3_d v3d_divs(v3_d v, double s) {
    return (v3_d){
        .x=(v.x / s),
        .y=(v.y / s),
        .z=(v.z / s),
    };
}

static inline v3_d v3d_cross(v3_d a, v3_d b) {
    return (v3_d){
        .x = (a.y * b.z) - (a.z * b.y),
        .y = (a.z * b.x) - (a.x * b.z),
        .z = (a.x * b.y) - (a.y * b.x),
    };
}

static inline v3_d v3d_proj(v3_d a, v3_d b) {
    double s = v3d_dot(a, b) / v3d_dot(b, b);
    return v3d_muls(b, s);
}

static inline v3_d v3d_norm(v3_d v) {
    return v3d_divs(v, v3d_mag(v));
}

static inline v3_d v3d_lerp(v3_d a, v3_d b, double t) {
    return v3d_add(v3d_muls(a, 1.0 - t), v3d_muls(b, t));
}


static inline double v3d_mag(v3_d v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

static inline double v3d_fastmag(v3_d v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

static inline double v3d_dot(v3_d a, v3_d b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline double v3d_angle(v3_d a, v3_d b) {
    return acosf(v3d_dot(a, b) / (v3d_mag(a) * v3d_mag(b)));
}

static inline int v3d_isZero(v3_d v) {
    return v3d_fastmag(v) < EPSILON_D;
}


// 4D Vectors Implementaions
// -------------------------

static inline v4_d v4d(double x, double y, double z, double w) {
    return (v4_d){ .x=x, .y=y, .z=z, .w=w };
}

static inline v4_d v4dInit(double i) {
    return (v4_d){ .x=i, .y=i, .z=i, .w=i };
}


static inline v4_d v4d_add(v4_d a, v4_d b) {
    return (v4_d){
        .x=(a.x + b.x),
        .y=(a.y + b.y),
        .z=(a.z + b.z),
        .w=(a.w + b.w),
    };
}

static inline v4_d v4d_adds(v4_d v, double s) {
    return (v4_d){
        .x=(v.x + s),
        .y=(v.y + s),
        .z=(v.z + s),
        .w=(v.w + s),
    };
}

static inline v4_d v4d_sub(v4_d a, v4_d b) {
    return (v4_d){
        .x=(a.x - b.x),
        .y=(a.y - b.y),
        .z=(a.z - b.z),
        .w=(a.w - b.w),
    };
}

static inline v4_d v4d_subs(v4_d v, double s) {
    return (v4_d){
        .x=(v.x - s),
        .y=(v.y - s),
        .z=(v.z - s),
        .w=(v.w - s),
    };
}

static inline v4_d v4d_mul(v4_d a, v4_d b) {
    return (v4_d){
        .x=(a.x * b.x),
        .y=(a.y * b.y),
        .z=(a.z * b.z),
        .w=(a.w * b.w),
    };
}

static inline v4_d v4d_muls(v4_d v, double s) {
    return (v4_d){
        .x=(v.x * s),
        .y=(v.y * s),
        .z=(v.z * s),
        .w=(v.w * s),
    };
}

static inline v4_d v4d_div(v4_d a, v4_d b) {
    return (v4_d){
        .x=(a.x / b.x),
        .y=(a.y / b.y),
        .z=(a.z / b.z),
        .w=(a.w / b.w),
    };
}

static inline v4_d v4d_divs(v4_d v, double s) {
    return (v4_d){
        .x=(v.x / s),
        .y=(v.y / s),
        .z=(v.z / s),
        .w=(v.w / s),
    };
}


static inline v4_d v4d_proj(v4_d a, v4_d b) {
    double s = v4d_dot(a, b) / v4d_dot(b, b);
    return v4d_muls(b, s);
}

static inline v4_d v4d_norm(v4_d v) {
    return v4d_divs(v, v4d_mag(v));
}

static inline v4_d v4d_normAxis(v4_d v) {
    return (v4_d){
        .angle = v.angle,
        .axis = v3d_norm(v.axis),
    };
}

static inline v4_d v4d_lerp(v4_d a, v4_d b, double t) {
    return v4d_add(v4d_muls(a, 1.0 - t), v4d_muls(b, t));
}


static inline double v4d_mag(v4_d v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
}

static inline double v4d_fastmag(v4_d v) {
    return v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w;
}

static inline double v4d_dot(v4_d a, v4_d b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static inline double v4d_angle(v4_d a, v4_d b) {
    return acosf(v4d_dot(a, b) / (v4d_mag(a) * v4d_mag(b)));
}

static inline int v4_isZero(v4_d v) {
    return v4d_fastmag(v) < EPSILON_D;
}



// Vector Casting Implementaions
// -----------------------------

static inline v2_d v3d_do_v2d(v3_d v) {
    return v.v2;
}

static inline v2_d v4d_to_v2d(v4_d v) {
    return v.v2;
}


static inline v3_d v2d_to_v3d(v2_d v) {
    return (v3_d){ .x=v.x, .y=v.y, .z=0.0 };
}

static inline v3_d v4d_to_v3d(v4_d v) {
    return v.v3;
}


static inline v4_d v2d_to_v4d(v2_d v) {
    return (v4_d){ .x=v.x, .y=v.y, .z=0.0, .w=0.0 };
}

static inline v4_d v3d_to_v4d(v3_d v) {
    return (v4_d){ .x=v.x, .y=v.y, .z=v.z, .w=0.0 };
}


// Matrix Functions and the math!
// ------------------------------


static inline m4x4_d m4dv(v4_d c0, v4_d c1, v4_d c2, v4_d c3) {
    return (m4x4_d){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}

static inline m4x4_d m4d(
        double x0, double x1, double x2, double x3,
        double y0, double y1, double y2, double y3,
        double z0, double z1, double z2, double z3,
        double w0, double w1, double w2, double w3) {
    return (m4x4_d){
        .x0=x0, .y0=y0, .z0=z0, .w0=w0,
        .x1=x1, .y1=y1, .z1=z1, .w1=w1,
        .x2=x2, .y2=y2, .z2=z2, .w2=w2,
        .x3=x3, .y3=y3, .z3=z3, .w3=w3,
    };
}

static inline m4x4_d m4dInit(double i) {
    return (m4x4_d){
        .x0=i, .y0=i, .z0=i, .w0=i,
        .x1=i, .y1=i, .z1=i, .w1=i,
        .x2=i, .y2=i, .z2=i, .w2=i,
        .x3=i, .y3=i, .z3=i, .w3=i,
    };
}

static inline m4x4_d m4d_identity(void) {
    return (m4x4_d){
        .x0=1.0, .y0=0.0, .z0=0.0, .w0=0.0,
        .x1=0.0, .y1=1.0, .z1=0.0, .w1=0.0,
        .x2=0.0, .y2=0.0, .z2=1.0, .w2=0.0,
        .x3=0.0, .y3=0.0, .z3=0.0, .w3=1.0,
    };
}

static inline m4x4_d m4d_translation(v3_d offset) {
    return (m4x4_d){
        .x0=1.0,     .y0=0.0,     .z0=0.0,     .w0=0.0,
        .x1=0.0,     .y1=1.0,     .z1=0.0,     .w1=0.0,
        .x2=0.0,     .y2=0.0,     .z2=1.0,     .w2=0.0,
        .x3=offset.x, .y3=offset.y, .z3=offset.z, .w3=1.0,
    };
}

static inline m4x4_d m4d_scaling(v3_d scale) {
    return (m4x4_d){
        .x0=scale.x, .y0=0.0,    .z0=0.0,    .w0=0.0,
        .x1=0.0,    .y1=scale.y, .z1=0.0,    .w1=0.0,
        .x2=0.0,    .y2=0.0,    .z2=scale.z, .w2=0.0,
        .x3=0.0,    .y3=0.0,    .z3=0.0,    .w3=1.0,
    };
}


static inline m4x4_d m4_perspective(double fovy, double aratio, double near, double far) {
    double f = 1.0 / tanf(fovy / 2.0);

    m4x4_d result = m4d_scaling((v3_d){
        .x = f / aratio,
        .y = f,
        .z = (far + near) / (near - far),
    });

    result.z3 = (2.0 * far * near) / (near - far);
    result.w2 = -1.0;

    return result;
}

static inline m4x4_d m4d_ortho(
        double left, double right,
        double bottom, double top,
        double back, double front) {

    m4x4_d result = m4d_scaling((v3_d){
            .x = 2.0 / (right - left),
            .y = 2.0 / (top - bottom),
            .z = 2.0 / (back - front),
    });

    result.x3 = -(right + left) / (right - left);
    result.y3 = -(top + bottom) / (top - bottom);
    result.z3 = -(back + front) / (back - front);

    return result;
}


static inline m4x4_d m4d_transpose(m4x4_d m) {
    return (m4x4_d){
        .x0=m.x0, .y0=m.x1, .z0=m.x2, .w0=m.x3,
        .x1=m.y0, .y1=m.y1, .z1=m.y2, .w1=m.y3,
        .x2=m.z0, .y2=m.z1, .z2=m.z2, .w2=m.z3,
        .x3=m.w0, .y3=m.w1, .z3=m.w2, .w3=m.w3,
    };
}

static inline m4x4_d m4d_add(m4x4_d a, m4x4_t b) {
    return (m4x4_d){
        .x0 = a.x0 + b.x0, .x1 = a.x1 + b.x1, .x2 = a.x2 + b.x2, .x3 = a.x3 + b.x3,
        .y0 = a.y0 + b.y0, .y1 = a.y1 + b.y1, .y2 = a.y2 + b.y2, .y3 = a.y3 + b.y3,
        .z0 = a.z0 + b.z0, .z1 = a.z1 + b.z1, .z2 = a.z2 + b.z2, .z3 = a.z3 + b.z3,
        .w0 = a.w0 + b.w0, .w1 = a.w1 + b.w1, .w2 = a.w2 + b.w2, .w3 = a.w3 + b.w3,
    };
}

static inline m4x4_d m4d_adds(m4x4_d m, double s) {
    return (m4x4_d){
        .x0 = m.x0 + s, .x1 = m.x1 + s, .x2 = m.x2 + s, .x3 = m.x3 + s,
        .y0 = m.y0 + s, .y1 = m.y1 + s, .y2 = m.y2 + s, .y3 = m.y3 + s,
        .z0 = m.z0 + s, .z1 = m.z1 + s, .z2 = m.z2 + s, .z3 = m.z3 + s,
        .w0 = m.w0 + s, .w1 = m.w1 + s, .w2 = m.w2 + s, .w3 = m.w3 + s,
    };
}

static inline m4x4_d m4d_sub(m4x4_d a, m4x4_d b) {
    return (m4x4_d){
        .x0 = a.x0 - b.x0, .x1 = a.x1 - b.x1, .x2 = a.x2 - b.x2, .x3 = a.x3 - b.x3,
        .y0 = a.y0 - b.y0, .y1 = a.y1 - b.y1, .y2 = a.y2 - b.y2, .y3 = a.y3 - b.y3,
        .z0 = a.z0 - b.z0, .z1 = a.z1 - b.z1, .z2 = a.z2 - b.z2, .z3 = a.z3 - b.z3,
        .w0 = a.w0 - b.w0, .w1 = a.w1 - b.w1, .w2 = a.w2 - b.w2, .w3 = a.w3 - b.w3,
    };
}

static inline m4x4_d m4d_subs(m4x4_d m, double s) {
    return (m4x4_d){
        .x0 = m.x0 - s, .x1 = m.x1 - s, .x2 = m.x2 - s, .x3 = m.x3 - s,
        .y0 = m.y0 - s, .y1 = m.y1 - s, .y2 = m.y2 - s, .y3 = m.y3 - s,
        .z0 = m.z0 - s, .z1 = m.z1 - s, .z2 = m.z2 - s, .z3 = m.z3 - s,
        .w0 = m.w0 - s, .w1 = m.w1 - s, .w2 = m.w2 - s, .w3 = m.w3 - s,
    };
}

static inline m4x4_d m4d_mul(m4x4_d a, m4x4_d b) {
    v4_d c0 = {
        .x = a.x0*b.x0 + a.x1*b.y0 + a.x2*b.z0 + a.x3*b.w0,
        .y = a.y0*b.x0 + a.y1*b.y0 + a.y2*b.z0 + a.y3*b.w0,
        .z = a.z0*b.x0 + a.z1*b.y0 + a.z2*b.z0 + a.z3*b.w0,
        .w = a.w0*b.x0 + a.w1*b.y0 + a.w2*b.z0 + a.w3*b.w0,
	};

    v4_d c1 = {
        .x = a.x0*b.x1 + a.x1*b.y1 + a.x2*b.z1 + a.x3*b.w1,
        .y = a.y0*b.x1 + a.y1*b.y1 + a.y2*b.z1 + a.y3*b.w1,
        .z = a.z0*b.x1 + a.z1*b.y1 + a.z2*b.z1 + a.z3*b.w1,
        .w = a.w0*b.x1 + a.w1*b.y1 + a.w2*b.z1 + a.w3*b.w1,
	};

    v4_d c2 = {
        .x = a.x0*b.x2 + a.x1*b.y2 + a.x2*b.z2 + a.x3*b.w2,
        .y = a.y0*b.x2 + a.y1*b.y2 + a.y2*b.z2 + a.y3*b.w2,
        .z = a.z0*b.x2 + a.z1*b.y2 + a.z2*b.z2 + a.z3*b.w2,
        .w = a.w0*b.x2 + a.w1*b.y2 + a.w2*b.z2 + a.w3*b.w2,
	};

    v4_d c3 = {
        .x = a.x0*b.x3 + a.x1*b.y3 + a.x2*b.z3 + a.x3* b.w3,
        .y = a.y0*b.x3 + a.y1*b.y3 + a.y2*b.z3 + a.y3* b.w3,
        .z = a.z0*b.x3 + a.z1*b.y3 + a.z2*b.z3 + a.z3* b.w3,
        .w = a.w0*b.x3 + a.w1*b.y3 + a.w2*b.z3 + a.w3* b.w3,
	};

    return (m4x4_d){ .c0=c0, .c1=c1, .c2=c2, .c3=c3 };
}


static inline m4x4_d m4d_muls(m4x4_d m, double s) {
    return (m4x4_d){
        .x0=m.x0 * s, .y0=m.y0 * s, .z0=m.z0 * s, .w0=m.w0 * s,
        .x1=m.x1 * s, .y1=m.y1 * s, .z1=m.z1 * s, .w1=m.w1 * s,
        .x2=m.x2 * s, .y2=m.y2 * s, .z2=m.z2 * s, .w2=m.w2 * s,
        .x3=m.x3 * s, .y3=m.y3 * s, .z3=m.z3 * s, .w3=m.w3 * s,
    };
}


static inline v4_d m4d_mulv2(m4x4_t m, v2_d v) {
    return (v4_d){
        .x = v.x * m.x0 + v.y * m.x1,
        .y = v.x * m.y0 + v.y * m.y1,
        .z = v.x * m.z0 + v.y * m.z1,
        .w = v.x * m.w0 + v.y * m.w1,
    };
}

static inline v4_d m4d_mulv3(m4x4_t m, v3_d v) {
    return (v4_d){
        .x = v.x * m.x0 + v.y * m.x1 + v.z * m.x2,
        .y = v.x * m.y0 + v.y * m.y1 + v.z * m.y2,
        .z = v.x * m.z0 + v.y * m.z1 + v.z * m.z2,
        .w = v.x * m.w0 + v.y * m.w1 + v.z * m.w2,
    };
}

static inline v4_d m4d_mulv4(m4x4_t m, v4_d v) {
    return (v4_d){
        .x = v.x * m.x0 + v.y * m.x1 + v.z * m.x2 + v.w * m.x3,
        .y = v.x * m.y0 + v.y * m.y1 + v.z * m.y2 + v.w * m.y3,
        .z = v.x * m.z0 + v.y * m.z1 + v.z * m.z2 + v.w * m.z3,
        .w = v.x * m.w0 + v.y * m.w1 + v.z * m.w2 + v.w * m.w3,
    };
}

static inline v4_d m4d_v2mul(v2_d v, m4x4_t m) {
	return (v4_d){
        .x = v.x * m.x0 + v.y * m.y0,
        .y = v.x * m.x1 + v.y * m.y1,
        .z = v.x * m.x2 + v.y * m.y2,
        .w = v.x * m.x3 + v.y * m.y3,
	};
}

static inline v4_d m4d_v3mul(v3_d v, m4x4_t m) {
	return (v4_d){
        .x = v.x * m.x0 + v.y * m.y0 + v.z * m.z0,
        .y = v.x * m.x1 + v.y * m.y1 + v.z * m.z1,
        .z = v.x * m.x2 + v.y * m.y2 + v.z * m.z2,
        .w = v.x * m.x3 + v.y * m.y3 + v.z * m.z3,
	};
}

static inline v4_d m4d_v4mul(v4_d v, m4x4_t m) {
	return (v4_d){
        .x = v.x * m.x0 + v.y * m.y0 + v.z * m.z0 + v.w * m.w0,
        .y = v.x * m.x1 + v.y * m.y1 + v.z * m.z1 + v.w * m.w1,
        .z = v.x * m.x2 + v.y * m.y2 + v.z * m.z2 + v.w * m.w2,
        .w = v.x * m.x3 + v.y * m.y3 + v.z * m.z3 + v.w * m.w3,
	};
}

#endif

#endif
