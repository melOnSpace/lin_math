#ifndef LIN_MATH
#define LIN_MATH

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

#define LIN_PI_F         3.141592653f
#define LIN_TAU_F        6.283185307f
#define LIN_E_F          2.718281828f
#define LIN_EPSILON_F    1.192092896e-07f

#define LIN_SQRT_TWO_F   1.414213562f
#define LIN_SQRT_THREE_F 1.732050807f
#define LIN_SQRT_FIVE_F  2.236067977f

#define LIN_LN2_F        0.693147180f
#define LIN_LN10_F       2.302585092f


// A single-header linear algerba library for C. Supports 2D, 3D,
// and 4D vectors. There is only a 4x4 matrix, and quaternions are
// supported as well. Evert function is staticly inlined. This might
// be a concern for bloating the final exe, but oh well. 
//
// By default the library only will include function definitions.
// Actual implementations of the functions must be enabled before
// including the header file in a given .c file. This can be done
// with #define LIN_MATH_IMPLEMENTATION
//
// 32-bit floats are the default for lin_math. Doubles are supported
// but also must be enabled with another macro. #define LIN_DOUBLE
// Said macro only enables double function definitions. Another
// macro must be used to enable implementation for doubles.
// Said macro is #define LIN_DOUBLE_IMPLEMENTATION
//
// Here is all the include macros and what they do:
//
// Enables 32-bit float function implementations
// #define LIN_MATH_IMPLEMENTATION
//
// Enables all 64-bit double function definitions
// #define LIN_DOUBLE
//
// Enables 64-bit double function implementations
// #define LIN_DOUBLE_IMPLEMENTATION
//
// Enables all of the above
// #define LIN_ENABLE_ALL
//
// All given Vectors can be 2D, 3D, or 4D. They are defined as
// v#_t. For example a 3D Vector is a v3_t; a
// 2D Vector is v2_t. A given vector is is a struct of N floats or
// doubles. All vectors can be accessed in different ways for a
// given context. See the type definitions below for said context.
// 
// The matrix type is 4x4; thus contains 16 floats/doubles.
// Internally they are stored in COLUMN-MAJOR ORDER. If you are
// anything like me, column-major and row-major are always weirdly
// confusing. Collum-major in this context means that a given
// matrix is stored as an array of four 4D vectors. Hope that helps
//
// With matrices and quaternions order of multiplication matters.
// For a given matrix multiplication the order is left-to-right.
// For example, if one wants to first scale, then rotate, then
// translate -> final matrix = translate * rotate * scale
//     or m4x4_t final = m4_mul(m4_mul(translate, rotate), scale);
//     or m4x4_t final = m4_product(3, translate, rotate, scale);
// 
// The same is true of quaternions. They are left-to-right. For
// example, for a rotation along the x-axis, then y-axis, then
// the z-axis -> final rotation = z-axis * y-axis * x-axis
//     or qt_t final = qt_mul(qt_mul(z_axis, y_axis), x_axis);
//     or qt_t final = qt_product(3, z_axis, y_axis, x_axis);
//
// OpenGL expects all matrix types to be in column-major order. 
// If you want a matrix to be in row-order then a transpose
// function is provided. Just keep in mind that OpenGL will have to
// be told to transpose the matrix. When passing a matrix to a
// uniform it should look like this: 
//     glUniformMatrix4fv(location, 1, GL_FALSE, &matrix.m[0][0]);
//         use GL_TRUE if the matrix has been transposed
// ----------------------------------------------------------------

#define LIN_ENABLE_ALL
#ifdef LIN_ENABLE_ALL
#define LIN_MATH_IMPLEMENTATION
#define LIN_DOUBLE
#define LIN_DOUBLE_IMPLEMENTATION
#endif

#define _packed_ __attribute__((__packed__))

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
    struct _packed_ { float x, y, z, w; };                 // Vector
    struct _packed_ { float u, v, t, s; };                 // Texture (4D textures?)
    struct _packed_ { float r, g, b, a; };                 // Color
    struct _packed_ { float real, i, j, k; };              // Quaternion (imaginary)
    struct _packed_ { float width, height, depth, time; }; // 4D Sizes ¯\_(ツ)_/¯
    struct _packed_ { v3_t axis; float angle; };           // Axis Angle

    v2_t v2; v3_t v3; // Casting
    float data[4];    // Raw Data
} v4_t;

typedef union {
    struct _packed_ { float r, i, j, k; }; // Traditional Representation
    struct _packed_ { float w; v3_t v; };  // Real and Vector Parts

    v4_t v4;       // Cast to Vector
    float data[4]; // Raw Data
} qt_t;

typedef union {
    struct { v4_t c0, c1, c2, c3; }; // Matrix Collums
    struct {
        float x0, y0, z0, w0;
        float x1, y1, z1, w1;
        float x2, y2, z2, w2;
        float x3, y3, z3, w3;
    }; // Matrix elements

    float m[4][4];  // Column Major
    float data[16]; // Raw Data
} m4x4_t;

// Helper Functions! These are just useful
// to have on hand even if they do not use
// vectors or matrixies
// ---------------------------------------

static inline float fast_rsqrtf(float number);
static inline float lerpf(float a, float b, float t);

// 2D Vector functions! These are just the
// decorations
// ---------------------------------------

static inline v2_t vec2(float x, float y); /* Returns a 2D Vector */
static inline v2_t vec2Init(float i); /* Returns vector initalized to i */

static inline v2_t v2_add(v2_t a, v2_t b);   /* Pair-wise addition */
static inline v2_t v2_adds(v2_t v, float s); /* Adds s to all fields */
static inline v2_t v2_sub(v2_t a, v2_t b);   /* Pair-wise subtraction */
static inline v2_t v2_subs(v2_t v, float s); /* Subs s to all fields */
static inline v2_t v2_mul(v2_t a, v2_t b);   /* Pair-wise multiplication */
static inline v2_t v2_muls(v2_t v, float s); /* Muls s to all fields */
static inline v2_t v2_div(v2_t a, v2_t b);   /* Pair-wise division */
static inline v2_t v2_divs(v2_t v, float s); /* Divs s to all fields */

static inline v2_t v2_sum(int num, ...);                        /* Variatic addition */
static inline v2_t v2_sum_array(size_t len, v2_t* array);       /* Sumation of an array */
static inline v2_t v2_differnce(int num, ...);                  /* Variatic subtraction */
static inline v2_t v2_differnce_array(size_t len, v2_t* array); /* Difference of an array */
static inline v2_t v2_product(int num, ...);                    /* Variatic multiplication */
static inline v2_t v2_product_array(size_t len, v2_t* array);   /* Product of an array */
static inline v2_t v2_quotient(int num, ...);                   /* Variatic division */
static inline v2_t v2_quotient_array(size_t len, v2_t* array);  /* Quotient of an array */

static inline v2_t v2_proj(v2_t a, v2_t b);          /* Project a onto b */
static inline v2_t v2_norm(v2_t v);                  /* Normalize v */
static inline v2_t v2_fastnorm(v2_t v);              /* Fast Normalize. Same as norm but with some margin of error */
static inline v2_t v2_lerp(v2_t a, v2_t b, float t); /* linear interp */

static inline float v2_mag(v2_t v);              /* Magnitude of v */
static inline float v2_fastmag(v2_t v);          /* Magnitude squared of v */
static inline float v2_dist(v2_t a, v2_t b);     /* Magnitude of v2_sub(a, b) */
static inline float v2_fastdist(v2_t a, v2_t b); /* Magnitude^2 of v2_sub(a, b) */
static inline float v2_dot(v2_t a, v2_t b);      /* Dot product of a and b */
static inline float v2_angle(v2_t a, v2_t b);    /* Angle (in rads) betwen a and b */
static inline int v2_isZero(v2_t v);             /* Checks if v is zero */

// Prints a vector. Works with both v2_t & v2_d
#define v2_print(v) (printf("[%f, %f]", (v).x, (v).y))
// Prints a vector. Works with both v2_t & v2_d
#define v2_println(v) (printf("[%f, %f]\n", (v).x, (v).y))

// 3D Vector functions! These are just the
// decorations
// ---------------------------------------

static inline v3_t v3(float x, float y, float z); /* Returns a 3D Vector */
static inline v3_t v3Init(float i); /* Returns vector initalized to i  */

static inline v3_t v3_add(v3_t a, v3_t b);   /* Pair-wise addition */
static inline v3_t v3_adds(v3_t v, float s); /* Adds s to all fields */
static inline v3_t v3_sub(v3_t a, v3_t b);   /* Pair-wise subtraction */
static inline v3_t v3_subs(v3_t v, float s); /* Subs s to all fields */
static inline v3_t v3_mul(v3_t a, v3_t b);   /* Pair-wise multiplication */
static inline v3_t v3_muls(v3_t v, float s); /* Muls s to all fields */
static inline v3_t v3_div(v3_t a, v3_t b);   /* Pair-wise division */
static inline v3_t v3_divs(v3_t v, float s); /* Divs s to all fields */

static inline v3_t v3_sum(int num, ...);                        /* Variatic addition */
static inline v3_t v3_sum_array(size_t len, v3_t* array);       /* Sumation of an array */
static inline v3_t v3_differnce(int num, ...);                  /* Variatic subtraction */
static inline v3_t v3_differnce_array(size_t len, v3_t* array); /* Difference of an array */
static inline v3_t v3_product(int num, ...);                    /* Variatic multiplication */
static inline v3_t v3_product_array(size_t len, v3_t* array);   /* Product of an array */
static inline v3_t v3_quotient(int num, ...);                   /* Variatic division */
static inline v3_t v3_quotient_array(size_t len, v3_t* array);  /* Quotient of an array */

static inline v3_t v3_cross(v3_t a, v3_t b);         /* Cross-Product of a and b */
static inline v3_t v3_proj(v3_t a, v3_t b);          /* Project a onto b */
static inline v3_t v3_norm(v3_t v);                  /* Normalize v */
static inline v3_t v3_fastnorm(v3_t v);              /* Fast Normalize. Same as norm but with some margin of error */
static inline v3_t v3_lerp(v3_t a, v3_t b, float t); /* linear interp */

static inline float v3_mag(v3_t v);              /* Magnitude of v */
static inline float v3_fastmag(v3_t v);          /* Magnitude squared of v */
static inline float v3_dist(v3_t a, v3_t b);     /* Magnitude of v3_sub(a, b) */
static inline float v3_fastdist(v3_t a, v3_t b); /* Magnitude^2 of v3_sub(a, b) */
static inline float v3_dot(v3_t a, v3_t b);      /* Dot Product of a and b */
static inline float v3_angle(v3_t a, v3_t b);    /* Angle between a and b */
static inline int v3_isZero(v3_t v);             /* Checks if v is zero */

// Prints a vector. Works with both v3_t & v3_d
#define v3_print(v) (printf("[%f, %f, %f]", (v).x, (v).y, (v).z))
// Prints a vector. Works with both v3_t & v3_d
#define v3_println(v) (printf("[%f, %f, %f]\n", (v).x, (v).y, (v).z))

// 4D Vector functions! These are just the
// decorations
// ---------------------------------------

static inline v4_t v4(float x, float y, float z, float w); /* Returns a 4D Vector */
static inline v4_t v4Init(float i); /* Returns vector initalized to i */

static inline v4_t v4_add(v4_t a, v4_t b);   /* Pair-wise addition */
static inline v4_t v4_adds(v4_t v, float s); /* Adds s to all fields */
static inline v4_t v4_sub(v4_t a, v4_t b);   /* Pair-wise subtraction */
static inline v4_t v4_subs(v4_t v, float s); /* Subs s to all fields */
static inline v4_t v4_mul(v4_t a, v4_t b);   /* Pair-wise multiplication */
static inline v4_t v4_muls(v4_t v, float s); /* Muls s to all fields */
static inline v4_t v4_div(v4_t a, v4_t b);   /* Pair-wise division */
static inline v4_t v4_divs(v4_t v, float s); /* Divs s to all fields */

static inline v4_t v4_sum(int num, ...);                        /* Variatic addition */
static inline v4_t v4_sum_array(size_t len, v4_t* array);       /* Sumation of an array */
static inline v4_t v4_differnce(int num, ...);                  /* Variatic subtraction */
static inline v4_t v4_differnce_array(size_t len, v4_t* array); /* Difference of an array */
static inline v4_t v4_product(int num, ...);                    /* Variatic multiplication */
static inline v4_t v4_product_array(size_t len, v4_t* array);   /* Product of an array */
static inline v4_t v4_quotient(int num, ...);                   /* Variatic division */
static inline v4_t v4_quotient_array(size_t len, v4_t* array);  /* Quotient of an array */

static inline v4_t v4_proj(v4_t a, v4_t b);          /* Project a onto b */
static inline v4_t v4_norm(v4_t v);                  /* Normalize v */
static inline v4_t v4_fastnorm(v4_t v);              /* Fast Normalize. Same as norm but with some margin of error */
static inline v4_t v4_normAxis(v4_t v);              /* Normalize only v.axis */
static inline v4_t v4_fastnormAxis(v4_t v);          /* Fast Normalize. Same as normAxis but with some margin of error */
static inline v4_t v4_lerp(v4_t a, v4_t b, float t); /* linear interp */

static inline float v4_mag(v4_t v);              /* Magnitude of v */
static inline float v4_fastmag(v4_t v);          /* Magnitude squared of v */
static inline float v4_dist(v4_t a, v4_t b);     /* Magnitude of v4_sub(a, b) */
static inline float v4_fastdist(v4_t a, v4_t b); /* Magnitude^2 of v4_sub(a, b) */
static inline float v4_dot(v4_t a, v4_t b);      /* Dot product of a and b */
static inline float v4_angle(v4_t a, v4_t b);    /* Angle between a and b */
static inline int v4_isZero(v4_t v);             /* Checks if v is zero */

// Prints a vector. Works with both v4_t & v4_d
#define v4_print(v) (printf("[%f, %f, %f, %f]", (v).x, (v).y, (v).z, (v).w))
// Prints a vector. Works with both v4_t & v4_d 
#define v4_println(v) (printf("[%f, %f, %f, %f]\n", (v).x, (v).y, (v).z, (v).w))

// Quaternion functions! These are just the
// decorations
// ---------------------------------------

static inline qt_t qt(float r, float i, float j, float k); /* Returns a quaternion */
static inline qt_t qtv(v4_t v); /* Returns a quaternion */
static inline qt_t qtInit(float i); /* Returns quaternion initalized to i */

static inline qt_t qt_identity(void); /* Returns 1 + 0i + 0j + 0k */
static inline qt_t qt_axis(float x, float y, float z, float angle); /* Axis-Angle to quaternion */
static inline qt_t qtv_axis(v4_t axis); /* Axis angle to quaternion */

static inline qt_t qt_add(qt_t a, qt_t b);   /* Pair-Wise addtion */
static inline qt_t qt_adds(qt_t q, float s); /* Adds s to all fields */
static inline qt_t qt_sub(qt_t a, qt_t b);   /* Pair-Wise subtraction */
static inline qt_t qt_subs(qt_t q, float s); /* Subs s to all fields */
static inline qt_t qt_mul(qt_t a, qt_t b);   /* Quaternion multiplication */
static inline qt_t qt_muls(qt_t q, float s); /* Muls s to all fields */
static inline qt_t qt_divs(qt_t q, float s); /* Divs s to all fields */

static inline qt_t qt_sum(int num, ...);                        /* Variatic addition */
static inline qt_t qt_sum_array(size_t len, qt_t* array);       /* Sumation of an array */
static inline qt_t qt_differnce(int num, ...);                  /* Variatic subtraction */
static inline qt_t qt_differnce_array(size_t len, qt_t* array); /* Difference of an array */
static inline qt_t qt_product(int num, ...);                    /* Variatic multiplication */
static inline qt_t qt_product_array(size_t len, qt_t* array);   /* Product of an array */

static inline qt_t qt_sqrt(qt_t q); /* Square-Root of a quaternion */
static inline qt_t qt_exp(qt_t q);  /* Exponential of a quaternion */
static inline qt_t qt_ln(qt_t q);   /* Natural Log of a quaternion */
static inline qt_t qt_pows(qt_t q, float s); /* Quaternion to the power of s */

static inline qt_t qt_conjugate(qt_t q);     /* q* = q.r - q.i - q.j - q.k */
static inline qt_t qt_inverse(qt_t q);       /* q^-1 = q* / qt_fastmag(q) */
static inline qt_t qt_cross(qt_t a, qt_t b); /* Cross product of a quaternion */
static inline qt_t qt_norm(qt_t q);          /* Normalize q */
static inline qt_t qt_fastnorm(qt_t q);      /* Fast Normalize. Same as norm but with some margin of error */
static inline qt_t qt_lerp(qt_t a, qt_t b, float t);  /* linear interp */
static inline qt_t qt_slerp(qt_t a, qt_t b, float t); /* spherical interp */

static inline float qt_mag(qt_t q);             /* Magnitude of q */
static inline float qt_fastmag(qt_t q);         /* Magnitude squared of q */
static inline float qt_geonorm(qt_t a, qt_t b); /* Geodesic Distance, i.e., angle between */
static inline float qt_dot(qt_t a, qt_t b);     /* Dot product */
static inline int qt_isZero(qt_t q);            /* Checks if q is zero */
static inline int qt_isUnit(qt_t q);            /* Checks if q is a unit quaternion (mag == 1) */
static inline int qt_isIdentity(qt_t q);        /* Checks if q == 1 + 0i + 0j + 0k */

// Prints a quaternion. Works with both qt_t & qt_d
#define qt_print(v) (printf("[%f, %fi, %fj, %fk]", (v).x, (v).y, (v).z, (v).w))
// Prints a quaternion. Works with both qt_t & qt_d 
#define qt_println(v) (printf("[%f, %fi, %fj, %fk]\n", (v).x, (v).y, (v).z, (v).w))

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

static inline m4x4_t m4v(v4_t c0, v4_t c1, v4_t c2, v4_t c3); /* Returns a 4x4 Matrix */
static inline m4x4_t m4(
        float x0, float x1, float x2, float x3,
        float y0, float y1, float y2, float y3,
        float z0, float z1, float z2, float z3,
        float w0, float w1, float w2, float w3); /* Returns a 4x4 Matrix */
static inline m4x4_t m4Init(float i); /* Returns a 4x4 Matrix initalized to i */

static inline m4x4_t m4_identity(void);                         /* Returns identity matrix */
static inline m4x4_t m4_translation(float x, float y, float z); /* Returns translation matrix */
static inline m4x4_t m4v_translation(v3_t offset);              /* Returns translation matrix */
static inline m4x4_t m4_scaling(float x, float y, float z);     /* Returns scaling matrix */
static inline m4x4_t m4v_scaling(v3_t scale);                   /* Returns scaling matrix */

static inline m4x4_t m4_xrot(float x); /* Returns rotation matrix for x-axis */
static inline m4x4_t m4_yrot(float y); /* Returns rotation matrix for y-axis */
static inline m4x4_t m4_zrot(float z); /* Returns rotation matrix for z-axis */
static inline m4x4_t m4_perspective(
        float fovy, float aratio,
        float near_plane, float far_plane); /* Returns perspective matrix */
static inline m4x4_t m4_ortho(
        float left, float right,
        float bottom, float top,
        float back, float front); /* Returns ortho matrix */

// Euler Rotation in order of X-Axis -> Y-Axis -> Z-Axis
static inline m4x4_t m4_eulerXYZ(float x, float y, float z);
// Euler Rotation in order of X-Axis -> Y-Axis -> Z-Axis
static inline m4x4_t m4v_eulerXYZ(v3_t angle);
// Euler Rotation in order of Z-Axis -> Y-Axis -> X-Axis
static inline m4x4_t m4_eulerZYX(float z, float y, float x);
// Euler Rotation in order of Z-Axis -> Y-Axis -> X-Axis
static inline m4x4_t m4v_eulerZYX(v3_t angle);

static inline m4x4_t m4_axis(float x, float y, float z, float angle); /* Axis-Angle to matrix */
static inline m4x4_t m4v_axis(v4_t axis); /* Axis-Angle to matrix */

static inline m4x4_t m4_quaternion(float r, float i, float j, float k); /* Quaternion to matrix */
static inline m4x4_t m4v_quaternion(qt_t q); /* Quaternion to matrix */

static inline m4x4_t m4_transpose(m4x4_t m); /* Switch between column and row major order */
static inline m4x4_t m4_add(m4x4_t a, m4x4_t b); /* Pair-Wise addition */
static inline m4x4_t m4_adds(m4x4_t m, float s); /* Adds s to all fields */
static inline m4x4_t m4_sub(m4x4_t a, m4x4_t b); /* Pair-Wise subtraction */
static inline m4x4_t m4_subs(m4x4_t m, float s); /* Subs s to all fields */
static inline m4x4_t m4_mul(m4x4_t a, m4x4_t b); /* Matrix multiplication */
static inline m4x4_t m4_muls(m4x4_t m, float s); /* Muls s to all fields */
static inline m4x4_t m4_divs(m4x4_t m, float s); /* Divs s to all fields */

static inline m4x4_t m4_sum(int num, ...);                          /* Variatic addition */
static inline m4x4_t m4_sum_array(size_t len, m4x4_t* array);       /* Sumation of an array */
static inline m4x4_t m4_differnce(int num, ...);                    /* Variatic subtraction */
static inline m4x4_t m4_differnce_array(size_t len, m4x4_t* array); /* Difference of an array */
static inline m4x4_t m4_product(int num, ...);                      /* Variatic multiplication */
static inline m4x4_t m4_product_array(size_t len, m4x4_t* array);   /* Product of an array */

// This is mostly a joke function. Like, why lerp a matrix?
static inline m4x4_t m4_lerp(m4x4_t a, m4x4_t b, float t);

static inline v4_t m4_mulv2(m4x4_t m, v2_t v); /* new_v = M*v */
static inline v4_t m4_mulv3(m4x4_t m, v3_t v); /* new_v = M*v */
static inline v4_t m4_mulv4(m4x4_t m, v4_t v); /* new_v = M*v */

static inline v4_t m4_v2mul(v2_t v, m4x4_t m); /* new_v = v*M */
static inline v4_t m4_v3mul(v3_t v, m4x4_t m); /* new_v = v*M */
static inline v4_t m4_v4mul(v4_t v, m4x4_t m); /* new_v = v*M */


#define MATRIX_PRINT_STR "\
.x0=%f .x1=%f .x2=%f .x3=%f\n\
.y0=%f .y1=%f .y2=%f .y3=%f\n\
.z0=%f .z1=%f .z2=%f .z3=%f\n\
.w0=%f .w1=%f .w2=%f .w3=%f\n"

// Prints a matrix. Works with both m4x4_t & m4x4_d
#define m4_println(m) (printf(MATRIX_PRINT_STR, \
            (m).x0, (m).x1, (m).x2, (m).x3,\
            (m).y0, (m).y1, (m).y2, (m).y3,\
            (m).z0, (m).z1, (m).z2, (m).z3,\
            (m).w0, (m).w1, (m).w2, (m).w3))

#define MATRIX_PRINT_COLOR "\
\033[33m.x0=\033[0m%f \033[33m.x1=\033[0m%f \033[33m.x2=\033[0m%f \033[33m.x3=\033[0m%f\n\
\033[33m.y0=\033[0m%f \033[33m.y1=\033[0m%f \033[33m.y2=\033[0m%f \033[33m.y3=\033[0m%f\n\
\033[33m.z0=\033[0m%f \033[33m.z1=\033[0m%f \033[33m.z2=\033[0m%f \033[33m.z3=\033[0m%f\n\
\033[33m.w0=\033[0m%f \033[33m.w1=\033[0m%f \033[33m.w2=\033[0m%f \033[33m.w3=\033[0m%f\n"

// Prints a matrix but with color. Works with both m4x4_t & m4x4_d
#define m4_printc(m) (printf(MATRIX_PRINT_COLOR, \
            (m).x0, (m).x1, (m).x2, (m).x3,\
            (m).y0, (m).y1, (m).y2, (m).y3,\
            (m).z0, (m).z1, (m).z2, (m).z3,\
            (m).w0, (m).w1, (m).w2, (m).w3))


#ifdef LIN_MATH_IMPLEMENTATION

// Help Function Implementaions
// ----------------------------

static inline float fast_rsqrtf(float number) {
    long i;
    float x2, y;
    const float threehalfs = 1.5f;

    x2 = number * 0.5;
    y = number;
    i = *(long*)&y;
    i = 0x5f3759df - (i >> i);
    y = *(float*)&i;
    y = y * (threehalfs - (x2 * y * y));
    // y = y * (threehalfs - (x2 * y * y)); // enable if need more accuracy

    return y;
}

static inline float lerpf(float a, float b, float t) {
    return a * (1.0f - t) + b * t;
}

// 2D Vectors Implementaions
// -------------------------

static inline v2_t vec2(float x, float y) {
    return (v2_t){ .x=x, .y=y };
}

static inline v2_t vec2Init(float i) {
    return (v2_t){ .x=i, .y=i };
}


static inline v2_t v2_add(v2_t a, v2_t b) {
    return (v2_t){
        .x = a.x + b.x,
        .y = a.y + b.y,
    };
}

static inline v2_t v2_adds(v2_t v, float s) {
    return (v2_t){
        .x = v.x + s,
        .y = v.y + s,
    };
}

static inline v2_t v2_sub(v2_t a, v2_t b) {
    return (v2_t){
        .x = a.x - b.x,
        .y = a.y - b.y,
    };
}

static inline v2_t v2_subs(v2_t v, float s) {
    return (v2_t){
        .x = v.x - s,
        .y = v.y - s,
    };
}

static inline v2_t v2_mul(v2_t a, v2_t b) {
    return (v2_t){
        .x = a.x * b.x,
        .y = a.y * b.y,
    };
}

static inline v2_t v2_muls(v2_t v, float s) {
    return (v2_t){
        .x = v.x * s,
        .y = v.y * s,
    };
}

static inline v2_t v2_div(v2_t a, v2_t b) {
    return (v2_t){
        .x = a.x / b.x,
        .y = a.y / b.y,
    };
}

static inline v2_t v2_divs(v2_t v, float s) {
    return (v2_t){
        .x = v.x / s,
        .y = v.y / s,
    };
}


static inline v2_t v2_sum(int num, ...) {
    va_list args;
    va_start(args, num);

    v2_t result = { .x=0.0f, .y=0.0f };
    for (int i = 0; i < num; ++i) {
        result = v2_add(result, va_arg(args, v2_t));
    }

    va_end(args);
    return result;
}

static inline v2_t v2_sum_array(size_t len, v2_t* array) {
    v2_t result = { .x=0.0f, .y=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v2_add(result, *array);
    }

    return result;
}

static inline v2_t v2_differnce(int num, ...) {
    va_list args;
    va_start(args, num);

    v2_t result = { .x=0.0f, .y=0.0f };
    for (int i = 0; i < num; ++i) {
        result = v2_sub(result, va_arg(args, v2_t));
    }

    va_end(args);
    return result;
}

static inline v2_t v2_differnce_array(size_t len, v2_t* array) {
    v2_t result = { .x=0.0f, .y=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v2_sub(result, *array);
    }

    return result;
}

static inline v2_t v2_product(int num, ...) {
    va_list args;
    va_start(args, num);

    v2_t result = { .x=0.0f, .y=0.0f };
    for (int i = 0; i < num; ++i) {
        result = v2_mul(result, va_arg(args, v2_t));
    }

    va_end(args);
    return result;
}

static inline v2_t v2_product_array(size_t len, v2_t* array) {
    v2_t result = { .x=0.0f, .y=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v2_mul(result, *array);
    }

    return result;
}

static inline v2_t v2_quotient(int num, ...) {
    va_list args;
    va_start(args, num);

    v2_t result = { .x=0.0f, .y=0.0f };
    for (int i = 0; i < num; ++i) {
        result = v2_div(result, va_arg(args, v2_t));
    }

    va_end(args);
    return result;
}

static inline v2_t v2_quotient_array(size_t len, v2_t* array) {
    v2_t result = { .x=0.0f, .y=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v2_div(result, *array);
    }

    return result;
}


static inline v2_t v2_proj(v2_t a, v2_t b) {
    float s = v2_dot(a, b) / v2_dot(b, b);
    return v2_muls(b, s);
}

static inline v2_t v2_norm(v2_t v) {
    return v2_divs(v, v2_mag(v));
}

static inline v2_t v2_fastnorm(v2_t v) {
    float fast_mag = fast_rsqrtf(v.x * v.x + v.y * v.y);
    return (v2_t){
        .x = v.x * fast_mag,
        .y = v.y * fast_mag,
    };
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

static inline float v2_dist(v2_t a, v2_t b) {
    return v2_mag(v2_sub(a, b));
}

static inline float v2_fastdist(v2_t a, v2_t b) {
    return v2_fastmag(v2_sub(a, b));
}

static inline float v2_dot(v2_t a, v2_t b) {
    return a.x * b.x + a.y * b.y;
}

static inline float v2_angle(v2_t a, v2_t b) {
    return acosf(v2_dot(a, b) / (v2_mag(a) * v2_mag(b)));
}

static inline int v2_isZero(v2_t v) {
    return v2_fastmag(v) < LIN_EPSILON_F;
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
        .x = a.x + b.x,
        .y = a.y + b.y,
        .z = a.z + b.z,
    };
}

static inline v3_t v3_adds(v3_t v, float s) {
    return (v3_t){
        .x = v.x + s,
        .y = v.y + s,
        .z = v.z + s,
    };
}

static inline v3_t v3_sub(v3_t a, v3_t b) {
    return (v3_t){
        .x = a.x - b.x,
        .y = a.y - b.y,
        .z = a.z - b.z,
    };
}

static inline v3_t v3_subs(v3_t v, float s) {
    return (v3_t){
        .x = v.x - s,
        .y = v.y - s,
        .z = v.z - s,
    };
}

static inline v3_t v3_mul(v3_t a, v3_t b) {
    return (v3_t){
        .x = a.x * b.x,
        .y = a.y * b.y,
        .z = a.z * b.z,
    };
}

static inline v3_t v3_muls(v3_t v, float s) {
    return (v3_t){
        .x = v.x * s,
        .y = v.y * s,
        .z = v.z * s,
    };
}

static inline v3_t v3_div(v3_t a, v3_t b) {
    return (v3_t){
        .x = a.x / b.x,
        .y = a.y / b.y,
        .z = a.z / b.z,
    };
}

static inline v3_t v3_divs(v3_t v, float s) {
    return (v3_t){
        .x = v.x / s,
        .y = v.y / s,
        .z = v.z / s,
    };
}


static inline v3_t v3_sum(int num, ...) {
    va_list args;
    va_start(args, num);

    v3_t result = { .x=0.0f, .y=0.0f, .z=0.0f };
    for (int i = 0; i < num; ++i) {
        result = v3_add(result, va_arg(args, v3_t));
    }

    va_end(args);
    return result;
}

static inline v3_t v3_sum_array(size_t len, v3_t* array) {
    v3_t result = { .x=0.0f, .y=0.0f, .z=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v3_add(result, *array);
    }

    return result;
}

static inline v3_t v3_differnce(int num, ...) {
    va_list args;
    va_start(args, num);

    v3_t result = { .x=0.0f, .y=0.0f, .z=0.0f };
    for (int i = 0; i < num; ++i) {
        result = v3_sub(result, va_arg(args, v3_t));
    }

    va_end(args);
    return result;
}

static inline v3_t v3_differnce_array(size_t len, v3_t* array) {
    v3_t result = { .x=0.0f, .y=0.0f, .z=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v3_sub(result, *array);
    }

    return result;
}

static inline v3_t v3_product(int num, ...) {
    va_list args;
    va_start(args, num);

    v3_t result = { .x=0.0f, .y=0.0f, .z=0.0f };
    for (int i = 0; i < num; ++i) {
        result = v3_mul(result, va_arg(args, v3_t));
    }

    va_end(args);
    return result;
}

static inline v3_t v3_product_array(size_t len, v3_t* array) {
    v3_t result = { .x=0.0f, .y=0.0f, .z=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v3_mul(result, *array);
    }

    return result;
}

static inline v3_t v3_quotient(int num, ...) {
    va_list args;
    va_start(args, num);

    v3_t result = { .x=0.0f, .y=0.0f, .z=0.0f };
    for (int i = 0; i < num; ++i) {
        result = v3_div(result, va_arg(args, v3_t));
    }

    va_end(args);
    return result;
}

static inline v3_t v3_quotient_array(size_t len, v3_t* array) {
    v3_t result = { .x=0.0f, .y=0.0f, .z=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v3_div(result, *array);
    }

    return result;
}


static inline v3_t v3_cross(v3_t a, v3_t b) {
    return (v3_t){
        .x = a.y * b.z - a.z * b.y,
        .y = a.z * b.x - a.x * b.z,
        .z = a.x * b.y - a.y * b.x,
    };
}

static inline v3_t v3_proj(v3_t a, v3_t b) {
    float s = v3_dot(a, b) / v3_dot(b, b);
    return v3_muls(b, s);
}

static inline v3_t v3_norm(v3_t v) {
    return v3_divs(v, v3_mag(v));
}

static inline v3_t v3_fastnorm(v3_t v) {
    float fast_mag = fast_rsqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    return (v3_t){
        .x = v.x * fast_mag,
        .y = v.y * fast_mag,
        .z = v.z * fast_mag,
    };
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

static inline float v3_dist(v3_t a, v3_t b) {
    return v3_mag(v3_sub(a, b));
}

static inline float v3_fastdist(v3_t a, v3_t b) {
    return v3_fastmag(v3_sub(a, b));
}

static inline float v3_dot(v3_t a, v3_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline float v3_angle(v3_t a, v3_t b) {
    return acosf(v3_dot(a, b) / (v3_mag(a) * v3_mag(b)));
}

static inline int v3_isZero(v3_t v) {
    return v3_fastmag(v) < LIN_EPSILON_F;
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
        .x = a.x + b.x,
        .y = a.y + b.y,
        .z = a.z + b.z,
        .w = a.w + b.w,
    };
}

static inline v4_t v4_adds(v4_t v, float s) {
    return (v4_t){
        .x = v.x + s,
        .y = v.y + s,
        .z = v.z + s,
        .w = v.w + s,
    };
}

static inline v4_t v4_sub(v4_t a, v4_t b) {
    return (v4_t){
        .x = a.x - b.x,
        .y = a.y - b.y,
        .z = a.z - b.z,
        .w = a.w - b.w,
    };
}

static inline v4_t v4_subs(v4_t v, float s) {
    return (v4_t){
        .x = v.x - s,
        .y = v.y - s,
        .z = v.z - s,
        .w = v.w - s,
    };
}

static inline v4_t v4_mul(v4_t a, v4_t b) {
    return (v4_t){
        .x = a.x * b.x,
        .y = a.y * b.y,
        .z = a.z * b.z,
        .w = a.w * b.w,
    };
}

static inline v4_t v4_muls(v4_t v, float s) {
    return (v4_t){
        .x = v.x * s,
        .y = v.y * s,
        .z = v.z * s,
        .w = v.w * s,
    };
}

static inline v4_t v4_div(v4_t a, v4_t b) {
    return (v4_t){
        .x = a.x / b.x,
        .y = a.y / b.y,
        .z = a.z / b.z,
        .w = a.w / b.w,
    };
}

static inline v4_t v4_divs(v4_t v, float s) {
    return (v4_t){
        .x = v.x / s,
        .y = v.y / s,
        .z = v.z / s,
        .w = v.w / s,
    };
}


static inline v4_t v4_sum(int num, ...) {
    va_list args;
    va_start(args, num);

    v4_t result = { .x=0.0f, .y=0.0f, .z=0.0f, .w=0.0f };
    for (int i = 0; i < num; ++i) {
        result = v4_add(result, va_arg(args, v4_t));
    }

    va_end(args);
    return result;
}

static inline v4_t v4_sum_array(size_t len, v4_t* array) {
    v4_t result = { .x=0.0f, .y=0.0f, .z=0.0f, .w=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v4_add(result, *array);
    }

    return result;
}

static inline v4_t v4_differnce(int num, ...) {
    va_list args;
    va_start(args, num);

    v4_t result = { .x=0.0f, .y=0.0f, .z=0.0f, .w=0.0f };
    for (int i = 0; i < num; ++i) {
        result = v4_sub(result, va_arg(args, v4_t));
    }

    va_end(args);
    return result;
}

static inline v4_t v4_differnce_array(size_t len, v4_t* array) {
    v4_t result = { .x=0.0f, .y=0.0f, .z=0.0f, .w=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v4_sub(result, *array);
    }

    return result;
}

static inline v4_t v4_product(int num, ...) {
    va_list args;
    va_start(args, num);

    v4_t result = { .x=0.0f, .y=0.0f, .z=0.0f, .w=0.0f };
    for (int i = 0; i < num; ++i) {
        result = v4_mul(result, va_arg(args, v4_t));
    }

    va_end(args);
    return result;
}

static inline v4_t v4_product_array(size_t len, v4_t* array) {
    v4_t result = { .x=0.0f, .y=0.0f, .z=0.0f, .w=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v4_mul(result, *array);
    }

    return result;
}

static inline v4_t v4_quotient(int num, ...) {
    va_list args;
    va_start(args, num);

    v4_t result = { .x=0.0f, .y=0.0f, .z=0.0f, .w=0.0f };
    for (int i = 0; i < num; ++i) {
        result = v4_div(result, va_arg(args, v4_t));
    }

    va_end(args);
    return result;
}

static inline v4_t v4_quotient_array(size_t len, v4_t* array) {
    v4_t result = { .x=0.0f, .y=0.0f, .z=0.0f, .w=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v4_div(result, *array);
    }

    return result;
}


static inline v4_t v4_proj(v4_t a, v4_t b) {
    float s = v4_dot(a, b) / v4_dot(b, b);
    return v4_muls(b, s);
}

static inline v4_t v4_norm(v4_t v) {
    return v4_divs(v, v4_mag(v));
}

static inline v4_t v4_fastnorm(v4_t v) {
    float fast_mag = fast_rsqrtf(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
    return (v4_t){
        .x = v.x * fast_mag,
        .y = v.y * fast_mag,
        .z = v.z * fast_mag,
        .w = v.w * fast_mag,
    };
}

static inline v4_t v4_normAxis(v4_t v) {
    return (v4_t){
        .angle = v.angle,
        .axis = v3_norm(v.axis),
    };
}

static inline v4_t v4_fastnormAxis(v4_t v) {
    float fast_mag = fast_rsqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    return (v4_t){
        .x = v.x * fast_mag,
        .y = v.y * fast_mag,
        .z = v.z * fast_mag,
        .angle = v.angle,
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

static inline float v4_dist(v4_t a, v4_t b) {
    return v4_mag(v4_sub(a, b));
}

static inline float v4_fastdist(v4_t a, v4_t b) {
    return v4_fastmag(v4_sub(a, b));
}

static inline float v4_dot(v4_t a, v4_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static inline float v4_angle(v4_t a, v4_t b) {
    return acosf(v4_dot(a, b) / (v4_mag(a) * v4_mag(b)));
}

static inline int v4_isZero(v4_t v) {
    return v4_fastmag(v) < LIN_EPSILON_F;
}


// Quaternion Implementaions
// -------------------------

static inline qt_t qt(float r, float i, float j, float k) {
    return (qt_t){ .r = r, .i = i, .j = j, .k = k };
}

static inline qt_t qtInit(float i) {
    return (qt_t){ .r = i, .i = i, .j = i, .k = i };
}


static inline qt_t qt_identity(void) {
    return (qt_t){ .r = 1.0f, .i = 0.0f, .j = 0.0f, .k = 0.0f };
}

static inline qt_t qt_axis(float x, float y, float z, float angle) {
    float sna = sinf(angle);
    return (qt_t){
        .r = cosf(angle),
        .i = x * sna,
        .j = y * sna,
        .k = z * sna,
    };
}

static inline qt_t qtv_axis(v4_t axis) {
    float sna = sinf(axis.angle);
    return (qt_t){
        .r = cosf(axis.angle),
        .i = axis.x * sna,
        .j = axis.y * sna,
        .k = axis.z * sna,
    };
}


static inline qt_t qt_add(qt_t a, qt_t b) {
    return (qt_t){
        .r = a.r + b.r,
        .i = a.i + b.i,
        .j = a.j + b.j,
        .k = a.k + b.k,
    };
}

static inline qt_t qt_adds(qt_t q, float s) {
    return (qt_t){
        .r = q.r + s,
        .i = q.i + s,
        .j = q.j + s,
        .k = q.k + s,
    };
}

static inline qt_t qt_sub(qt_t a, qt_t b) {
    return (qt_t){
        .r = a.r - b.r,
        .i = a.i - b.i,
        .j = a.j - b.j,
        .k = a.k - b.k,
    };
}

static inline qt_t qt_subs(qt_t q, float s) {
    return (qt_t){
        .r = q.r - s,
        .i = q.i - s,
        .j = q.j - s,
        .k = q.k - s,
    };
}

static inline qt_t qt_mul(qt_t a, qt_t b) {
    float r = (a.r * b.r) - (a.i * b.i) - (a.j * b.j) - (a.k * b.k);
    float i = (a.r * b.i) + (a.i * b.r) - (a.j * b.k) + (a.k * b.j);
    float j = (a.r * b.j) + (a.i * b.k) + (a.j * b.r) - (a.k * b.i);
    float k = (a.r * b.k) - (a.i * b.j) + (a.j * b.i) + (a.k * b.r);
    
    return (qt_t){ .r = r, .i = i, .j = j, .k = k };
}

static inline qt_t qt_muls(qt_t q, float s) {
    return (qt_t){
        .r = q.r * s,
        .i = q.i * s,
        .j = q.j * s,
        .k = q.k * s,
    };
}

static inline qt_t qt_divs(qt_t q, float s) {
    return (qt_t){
        .r = q.r / s,
        .i = q.i / s,
        .j = q.j / s,
        .k = q.k / s,
    };
}


static inline qt_t qt_sum(int num, ...) {
    va_list args;
    va_start(args, num);

    qt_t result = { .r=0.0f, .i=0.0f, .j=0.0f, .k=0.0f };
    for (int i = 0; i < num; ++i) {
        result = qt_add(result, va_arg(args, qt_t));
    }

    va_end(args);
    return result;
}

static inline qt_t qt_sum_array(size_t len, qt_t* array) {
    qt_t result = { .r=0.0f, .i=0.0f, .j=0.0f, .k=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = qt_add(result, *array);
    }

    return result;
}

static inline qt_t qt_differnce(int num, ...) {
    va_list args;
    va_start(args, num);

    qt_t result = { .r=0.0f, .i=0.0f, .j=0.0f, .k=0.0f };
    for (int i = 0; i < num; ++i) {
        result = qt_sub(result, va_arg(args, qt_t));
    }

    va_end(args);
    return result;
}

static inline qt_t qt_differnce_array(size_t len, qt_t* array) {
    qt_t result = { .r=0.0f, .i=0.0f, .j=0.0f, .k=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = qt_sub(result, *array);
    }

    return result;
}

static inline qt_t qt_product(int num, ...) {
    va_list args;
    va_start(args, num);

    qt_t result = { .r=0.0f, .i=0.0f, .j=0.0f, .k=0.0f };
    for (int i = 0; i < num; ++i) {
        result = qt_mul(result, va_arg(args, qt_t));
    }

    va_end(args);
    return result;
}

static inline qt_t qt_product_array(size_t len, qt_t* array) {
    qt_t result = { .r=0.0f, .i=0.0f, .j=0.0f, .k=0.0f };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = qt_mul(result, *array);
    }

    return result;
}


static inline qt_t qt_sqrt(qt_t q) {
    float qmag = sqrtf(q.r * q.r + q.i * q.i + q.j * q.j + q.k * q.k);

    float r = sqrtf((qmag + q.r) / 2.0f);
    v3_t v = v3_muls(v3_norm(q.v), (qmag - q.r) / 2.0f);
    return (qt_t){ .w = r, .v = v };
}

static inline qt_t qt_exp(qt_t q) {
    float vmag = v3_mag(q.v);
    
    float r = powf(LIN_E_F, q.r) * cosf(vmag);
    v3_t v = v3_muls(v3_divs(q.v, vmag), sinf(vmag));
    return (qt_t){ .w = r, .v = v };
}

static inline qt_t qt_ln(qt_t q) {
    float qmag = sqrtf(q.r * q.r + q.i * q.i + q.j * q.j + q.k * q.k);

    float r = logf(qmag);
    v3_t v = v3_muls(v3_norm(q.v), acosf(q.r / qmag));
    return (qt_t){ .w = r, .v = v };
}

static inline qt_t qt_pows(qt_t q, float s) {
    return qt_exp(qt_muls(qt_ln(q), s));
}


static inline qt_t qt_conjugate(qt_t q) {
    return (qt_t){ .r = q.r, .i = -q.i, .j = -q.j, .k = -q.k };
}

static inline qt_t qt_inverse(qt_t q) {
    float qmag = q.r * q.r + q.i * q.i + q.j * q.j + q.k * q.k;
    return (qt_t){
        .r=( q.r / qmag),
        .i=(-q.i / qmag),
        .j=(-q.j / qmag),
        .k=(-q.k / qmag),
    };
}

static inline qt_t qt_cross(qt_t a, qt_t b) {
    return (qt_t){
        .r = 0.0f,
        .i = a.j * b.k - a.k * b.j,
        .j = a.k * b.j - a.i * b.k,
        .k = a.i * b.j - a.j * b.i,
    };
}

static inline qt_t qt_norm(qt_t q) {
    float qmag = sqrtf(q.r * q.r + q.i * q.i + q.j * q.j + q.k * q.k);
    return (qt_t){
        .r = q.r / qmag,
        .i = q.i / qmag,
        .j = q.j / qmag,
        .k = q.k / qmag,
    };
}

static inline qt_t qt_fastnorm(qt_t q) {
    float fast_qmag = fast_rsqrtf(q.r * q.r + q.i * q.i + q.j * q.j + q.k * q.k);
    return (qt_t){
        .r = q.r / fast_qmag,
        .i = q.i / fast_qmag,
        .j = q.j / fast_qmag,
        .k = q.k / fast_qmag,
    };
}

static inline qt_t qt_lerp(qt_t a, qt_t b, float t) {
    return qt_add(qt_muls(a, 1.0f - t), qt_muls(b, t));
}

static inline qt_t qt_slerp(qt_t a, qt_t b, float t) {
    return qt_mul(a, qt_pows(qt_mul(qt_inverse(a), b), t));
}


static inline float qt_mag(qt_t q) {
    return sqrtf(q.r * q.r + q.i * q.i + q.j * q.j + q.k * q.k);
}

static inline float qt_fastmag(qt_t q) {
    return q.r * q.r + q.i * q.i + q.j * q.j + q.k * q.k;
}

static inline float qt_geonorm(qt_t a, qt_t b) {
    return qt_mag(qt_ln(qt_mul(qt_inverse(a), b)));
}

static inline float qt_dot(qt_t a, qt_t b) {
    return a.r * b.r + a.i * b.i + a.j * b.j + a.k * b.k;
}

static inline int qt_isZero(qt_t q) {
    return qt_fastmag(q) < LIN_EPSILON_F;
}

static inline int qt_isUnit(qt_t q) {
    return (qt_fastmag(q) - 1.0f) < LIN_EPSILON_F;
}

static inline int qt_isIdentity(qt_t q) {
    return (q.r - 1.0f) < LIN_EPSILON_F && v3_isZero(q.v);
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

static inline m4x4_t m4_translation(float x, float y, float z) {
    return (m4x4_t){
        .x0=1.0f, .y0=0.0f, .z0=0.0f, .w0=0.0f,
        .x1=0.0f, .y1=1.0f, .z1=0.0f, .w1=0.0f,
        .x2=0.0f, .y2=0.0f, .z2=1.0f, .w2=0.0f,
        .x3=x,    .y3=y,    .z3=z,    .w3=1.0f,
    };
}

static inline m4x4_t m4v_translation(v3_t offset) {
    return (m4x4_t){
        .x0=1.0f,     .y0=0.0f,     .z0=0.0f,     .w0=0.0f,
        .x1=0.0f,     .y1=1.0f,     .z1=0.0f,     .w1=0.0f,
        .x2=0.0f,     .y2=0.0f,     .z2=1.0f,     .w2=0.0f,
        .x3=offset.x, .y3=offset.y, .z3=offset.z, .w3=1.0f,
    };
}

static inline m4x4_t m4v_scaling(v3_t scale) {
    return (m4x4_t){
        .x0=scale.x, .y0=0.0f,    .z0=0.0f,    .w0=0.0f,
        .x1=0.0f,    .y1=scale.y, .z1=0.0f,    .w1=0.0f,
        .x2=0.0f,    .y2=0.0f,    .z2=scale.z, .w2=0.0f,
        .x3=0.0f,    .y3=0.0f,    .z3=0.0f,    .w3=1.0f,
    };
}

static inline m4x4_t m4_scaling(float x, float y, float z) {
    return (m4x4_t){
        .x0=x,    .y0=0.0f, .z0=0.0f, .w0=0.0f,
        .x1=0.0f, .y1=y,    .z1=0.0f, .w1=0.0f,
        .x2=0.0f, .y2=0.0f, .z2=z,    .w2=0.0f,
        .x3=0.0f, .y3=0.0f, .z3=0.0f, .w3=1.0f,
    };
}


static inline m4x4_t m4_xrot(float x) {
    float snx = sinf(x);
    float csx = cosf(x);

    return (m4x4_t){
        .x0=1.0f, .x1=0.0f, .x2=0.0f, .x3=0.0f,
        .y0=0.0f, .y1= csx, .y2=-snx, .y3=0.0f,
        .z0=0.0f, .z1= snx, .z2= csx, .z3=0.0f,
        .w0=0.0f, .w1=0.0f, .w2=0.0f, .w3=1.0f,
    };
}

static inline m4x4_t m4_yrot(float y) {
    float sny = sinf(y);
    float csy = cosf(y);

    return (m4x4_t){
        .x0= csy, .x1=0.0f, .x2= sny, .x3=0.0f,
        .y0=0.0f, .y1=1.0f, .y2=0.0f, .y3=0.0f,
        .z0=-sny, .z1=0.0f, .z2= csy, .z3=0.0f,
        .w0=0.0f, .w1=0.0f, .w2=0.0f, .w3=1.0f,
    };
}

static inline m4x4_t m4_zrot(float z) {
    float snz = sinf(z);
    float csz = cosf(z);

    return (m4x4_t){
        .x0= csz, .x1=-snz, .x2=0.0f, .x3=0.0f,
        .y0= snz, .y1= csz, .y2=0.0f, .y3=0.0f,
        .z0=0.0f, .z1=0.0f, .z2=1.0f, .z3=0.0f,
        .w0=0.0f, .w1=0.0f, .w2=0.0f, .w3=1.0f,
    };
}


static inline m4x4_t m4_perspective(float fovy, float aratio, float near_plane, float far_plane) {
    float f = 1.0f / tanf(fovy / 2.0f);

    m4x4_t result = m4_scaling(
            f / aratio,
            f,
            (far_plane + near_plane) / (near_plane - far_plane)
    );

    result.z3 = (2.0f * far_plane * near_plane) / (near_plane - far_plane);
    result.w2 = -1.0f;

    return result;
}

static inline m4x4_t m4_ortho(
        float left, float right,
        float bottom, float top,
        float back, float front) {

    m4x4_t result = m4_scaling(
            2.0f / (right - left),
            2.0f / (top - bottom),
            2.0f / (back - front)
    );

    result.x3 = -(right + left) / (right - left);
    result.y3 = -(top + bottom) / (top - bottom);
    result.z3 = -(back + front) / (back - front);

    return result;
}


// Euler Rotation in order of X-Axis -> Y-Axis -> Z-Axis
static inline m4x4_t m4_eulerXYZ(float x, float y, float z) {
    float snx = sinf(-x), sny = sinf(-y), snz = sinf(-z);
    float csx = cosf(-x), csy = cosf(-y), csz = cosf(-z);

    v4_t c0 = {
	    .x =  csy * csz,
	    .y = -csx * snz + snx * sny * csz,
	    .z =  snx * snz + csx * sny * csz,
	    .w =  0.0f,
    };

    v4_t c1 = {
	    .x =  csy * snz,
	    .y =  csx * csz + snx * sny * snz,
	    .z = -snx * csz + csx * sny * snz,
	    .w =  0.0f,
    };

    v4_t c2 = {
	    .x = -sny,
	    .y =  snx * csy,
	    .z =  csx * csy,
	    .w =  0.0f,
    };

    v4_t c3 = { .x = 0.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f };
    return (m4x4_t){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}

// Euler Rotation in order of X-Axis -> Y-Axis -> Z-Axis
static inline m4x4_t m4v_eulerXYZ(v3_t angle) {
    float snx = sinf(-angle.x), sny = sinf(-angle.y), snz = sinf(-angle.z);
    float csx = cosf(-angle.x), csy = cosf(-angle.y), csz = cosf(-angle.z);

    v4_t c0 = {
	    .x =  csy * csz,
	    .y = -csx * snz + snx * sny * csz,
	    .z =  snx * snz + csx * sny * csz,
	    .w =  0.0f,
    };

    v4_t c1 = {
	    .x =  csy * snz,
	    .y =  csx * csz + snx * sny * snz,
	    .z = -snx * csz + csx * sny * snz,
	    .w =  0.0f,
    };

    v4_t c2 = {
	    .x = -sny,
	    .y =  snx * csy,
	    .z =  csx * csy,
	    .w =  0.0f,
    };

    v4_t c3 = { .x = 0.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f };
    return (m4x4_t){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}

// Euler Rotation in order of Z-Axis -> Y-Axis -> X-Axis
static inline m4x4_t m4_eulerZYX(float z, float y, float x) {
    float snx = sinf(-x), sny = sinf(-y), snz = sinf(-z);
    float csx = cosf(-x), csy = cosf(-y), csz = cosf(-z);

    v4_t c0 = {
        .x = csz * csy,
	    .y = csy * snz,
	    .z =-sny,
	    .w =  0.0f,
    };

    v4_t c1 = {
	    .x = csz * sny * snx - csx * snz,
	    .y = csz * csx + snz * sny * snx,
	    .z = csy * snx,
	    .w =  0.0f,
    };

    v4_t c2 = {
	    .x = snz * snx + csz * csx * sny,
	    .y = csx * snz * sny - csz * snx,
	    .z = csy * csx,
	    .w =  0.0f,
    };
    

    v4_t c3 = { .x = 0.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f };
    return (m4x4_t){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}

// Euler Rotation in order of Z-Axis -> Y-Axis -> X-Axis
static inline m4x4_t m4v_eulerZYX(v3_t angle) {
    float snx = sinf(-angle.x), sny = sinf(-angle.y), snz = sinf(-angle.z);
    float csx = cosf(-angle.x), csy = cosf(-angle.y), csz = cosf(-angle.z);

    v4_t c0 = {
        .x = csz * csy,
	    .y = csy * snz,
	    .z =-sny,
    };

    v4_t c1 = {
	    .x = csz * sny * snx - csx * snz,
	    .y = csz * csx + snz * sny * snx,
	    .z = csy * snx,
    };

    v4_t c2 = {
	    .x = snz * snx + csz * csx * sny,
	    .y = csx * snz * sny - csz * snx,
	    .z = csy * csx,
    };
    

    v4_t c3 = { .x = 0.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f };
    return (m4x4_t){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}


static inline m4x4_t m4_axis(float x, float y, float z, float angle) {
    float csa = cosf(angle); float sna = sinf(angle);
    v4_t c0 = (v4_t){
        .x = csa + (x * x) * (1.0f - csa),
        .y = y * x * (1.0f - csa) + z * sna,
        .z = z * x * (1.0f - csa) - y * sna,
	};

    v4_t c1 = (v4_t){
        .x = x * y * (1.0f - csa) - z * sna,
        .y = csa + (y * y) * (1.0f - csa),
        .z = z * y * (1.0f - csa) + x * sna,
	};

    v4_t c2 = (v4_t){
        .x = x * z * (1.0f - csa) + y * sna,
        .y = y * z * (1.0f - csa) - x * sna,
        .z = csa + (z * z) * (1.0f - csa),
	};


    v4_t c3 = { .x = 0.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f };
    return (m4x4_t){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}

static inline m4x4_t m4v_axis(v4_t axis) {
    float csa = cosf(axis.angle); float sna = sinf(axis.angle);
    v4_t c0 = (v4_t){
        .x = csa + (axis.x * axis.x) * (1.0f - csa),
        .y = axis.y * axis.x * (1.0f - csa) + axis.z * sna,
        .z = axis.z * axis.x * (1.0f - csa) - axis.y * sna,
        .w = 0.0f,
	};

    v4_t c1 = (v4_t){
        .x = axis.x * axis.y * (1.0f - csa) - axis.z * sna,
        .y = csa + (axis.y * axis.y) * (1.0f - csa),
        .z = axis.z * axis.y * (1.0f - csa) + axis.x * sna,
        .w = 0.0f,
	};

    v4_t c2 = (v4_t){
        .x = axis.x * axis.z * (1.0f - csa) + axis.y * sna,
        .y = axis.y * axis.z * (1.0f - csa) - axis.x * sna,
        .z = csa + (axis.z * axis.z) * (1.0f - csa),
        .w = 0.0f,
	};


    v4_t c3 = { .x = 0.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f };
    return (m4x4_t){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}

static inline m4x4_t m4_quaternion(float r, float i, float j, float k) {
    v4_t c0 = {
        .x = 1.0f - 2.0f * (i * i + j * j),
        .y = 2.0f * (r * i + k * j),
        .z = 2.0f * (r * j - k * i),
        .w = 0.0f,
    };

    v4_t c1 = {
        .x = 2.0f * (r * i - k * j),
        .y = 1.0f - 2.0f * (r * r + j * j),
        .z = 2.0f * (i * j + k * r),
        .w = 0.0f,
    };

    v4_t c2 = {
        .x = 2.0f * (r * j + k * i),
        .y = 2.0f * (i * j - k * r),
        .z = 1.0f - 2.0f * (r * r + i * i),
        .w = 0.0f,
    };

    v4_t c3 = { .x = 0.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f };
	return (m4x4_t){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}

static inline m4x4_t m4v_quaternion(qt_t q) {
    v4_t c0 = {
        .x = 1.0f - 2.0f * (q.i * q.i + q.j * q.j),
        .y = 2.0f * (q.r * q.i + q.k * q.j),
        .z = 2.0f * (q.r * q.j - q.k * q.i),
        .w = 0.0f,
    };

    v4_t c1 = {
        .x = 2.0f * (q.r * q.i - q.k * q.j),
        .y = 1.0f - 2.0f * (q.r * q.r + q.j * q.j),
        .z = 2.0f * (q.i * q.j + q.k * q.r),
        .w = 0.0f,
    };

    v4_t c2 = {
        .x = 2.0f * (q.r * q.j + q.k * q.i),
        .y = 2.0f * (q.i * q.j - q.k * q.r),
        .z = 1.0f - 2.0f * (q.r * q.r + q.i * q.i),
        .w = 0.0f,
    };

    v4_t c3 = { .x = 0.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f };
	return (m4x4_t){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
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
        .x = a.x0*b.x3 + a.x1*b.y3 + a.x2*b.z3 + a.x3*b.w3,
        .y = a.y0*b.x3 + a.y1*b.y3 + a.y2*b.z3 + a.y3*b.w3,
        .z = a.z0*b.x3 + a.z1*b.y3 + a.z2*b.z3 + a.z3*b.w3,
        .w = a.w0*b.x3 + a.w1*b.y3 + a.w2*b.z3 + a.w3*b.w3,
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

static inline m4x4_t m4_divs(m4x4_t m, float s) {
    return (m4x4_t){
        .x0=m.x0 / s, .y0=m.y0 / s, .z0=m.z0 / s, .w0=m.w0 / s,
        .x1=m.x1 / s, .y1=m.y1 / s, .z1=m.z1 / s, .w1=m.w1 / s,
        .x2=m.x2 / s, .y2=m.y2 / s, .z2=m.z2 / s, .w2=m.w2 / s,
        .x3=m.x3 / s, .y3=m.y3 / s, .z3=m.z3 / s, .w3=m.w3 / s,
    };
}


static inline m4x4_t m4_sum(int num, ...) {
    va_list args;
    va_start(args, num);

    m4x4_t result = {0};
    for (int i = 0; i < num; ++i) {
        result = m4_add(result, va_arg(args, m4x4_t));
    }

    va_end(args);
    return result;
}

static inline m4x4_t m4_sum_array(size_t len, m4x4_t* array) {
    m4x4_t result = {0};
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = m4_add(result, *array);
    }

    return result;
}

static inline m4x4_t m4_differnce(int num, ...) {
    va_list args;
    va_start(args, num);

    m4x4_t result = {0};
    for (int i = 0; i < num; ++i) {
        result = m4_sub(result, va_arg(args, m4x4_t));
    }

    va_end(args);
    return result;
}

static inline m4x4_t m4_differnce_array(size_t len, m4x4_t* array) {
    m4x4_t result = {0};
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = m4_sub(result, *array);
    }

    return result;
}

static inline m4x4_t m4_product(int num, ...) {
    va_list args;
    va_start(args, num);

    m4x4_t result = {0};
    for (int i = 0; i < num; ++i) {
        result = m4_mul(result, va_arg(args, m4x4_t));
    }

    va_end(args);
    return result;
}

static inline m4x4_t m4_product_array(size_t len, m4x4_t* array) {
    m4x4_t result = {0};
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = m4_mul(result, *array);
    }

    return result;
}


static inline m4x4_t m4_lerp(m4x4_t a, m4x4_t b, float t) {
    return (m4x4_t) {
        .c0 = v4_lerp(a.c0, b.c0, t),
        .c1 = v4_lerp(a.c1, b.c1, t),
        .c2 = v4_lerp(a.c3, b.c2, t),
        .c3 = v4_lerp(a.c3, b.c3, t),
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

#define LIN_PI      3.14159265358979323846264338327950288
#define LIN_TAU     6.28318530717958647692528676655900576
#define LIN_E       2.71828182845904523536
#define LIN_EPSILON 2.2204460492503131e-016

#define LIN_SQRT_TWO   1.41421356237309504880168872420969808
#define LIN_SQRT_THREE 1.73205080756887729352744634150587236
#define LIN_SQRT_FIVE  2.23606797749978969640917366873127623

#define LIN_LN2        0.69314718055994530941723212145817656
#define LIN_LN10       2.30258509299404568401799145468436421


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
    struct _packed_ { double r, i, j, k; };
    struct _packed_ { double w; v3_d v; };

    v4_d v4;
    double data[4];
} qt_d;

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

// Helper Functions! These are just useful
// to have on hand even if they do not use
// vectors or matrixies
// ---------------------------------------

static inline double fast_rsqrt(double number);
static inline double lerp(double a, double b, double t);

// 2D Vector functions! These are just the
// decorations
// ---------------------------------------

static inline v2_d vec2d(double x, double y); /* Returns a 2D Vector */
static inline v2_d vec2dInit(double i);       /* Returns vector initalized to i */

static inline v2_d v2d_add(v2_d a, v2_d b);    /* Pair-wise addition */
static inline v2_d v2d_adds(v2_d v, double s); /* Adds s to all fields */
static inline v2_d v2d_sub(v2_d a, v2_d b);    /* Pair-wise subtraction */
static inline v2_d v2d_subs(v2_d v, double s); /* Subs s to all fields */
static inline v2_d v2d_mul(v2_d a, v2_d b);    /* Pair-wise multiplication */
static inline v2_d v2d_muls(v2_d v, double s); /* Muls s to all fields */
static inline v2_d v2d_div(v2_d a, v2_d b);    /* Pair-wise division */
static inline v2_d v2d_divs(v2_d v, double s); /* Divs s to all fields */

static inline v2_d v2d_sum(int num, ...);                   /* Variatic addition */
static inline v2_d v2da_sum(size_t len, v2_d* array);       /* Sumation of an array */
static inline v2_d v2d_differnce(int num, ...);             /* Variatic subtraction */
static inline v2_d v2da_differnce(size_t len, v2_d* array); /* Difference of an array */
static inline v2_d v2d_product(int num, ...);               /* Variatic multiplication */
static inline v2_d v2da_product(size_t len, v2_d* array);   /* Product of an array */
static inline v2_d v2d_quotient(int num, ...);              /* Variatic division */
static inline v2_d v2da_quotient(size_t len, v2_d* array);  /* Quotient of an array */

static inline v2_d v2d_proj(v2_d a, v2_d b);           /* Project a onto b */
static inline v2_d v2d_norm(v2_d v);                   /* Normalize v */
static inline v2_d v2d_fastnorm(v2_d v);               /* Fast Normalize. Same as norm but with some margin of error */
static inline v2_d v2d_lerp(v2_d a, v2_d b, double t); /* linear interp */

static inline double v2d_mag(v2_d v);              /* Magnitude of v */
static inline double v2d_fastmag(v2_d v);          /* Magnitude squared of v */
static inline double v2d_dist(v2_d a, v2_d b);     /* Magnitude of v2_sub(a, b) */
static inline double v2d_fastdist(v2_d a, v2_d b); /* Magnitude^2 of v2_sub(a, b) */
static inline double v2d_dot(v2_d a, v2_d b);      /* Dot product of a and b */
static inline double v2d_angle(v2_d a, v2_d b);    /* Angle (in rads) betwen a and b */
static inline int v2d_isZero(v2_d v);              /* Checks if v is zero */

// 3D Vector functions! These are just the
// decorations
// ---------------------------------------

static inline v3_d v3d(double x, double y, double z); /* Returns a 3D Vector */
static inline v3_d v3dInit(double i); /* Returns vector initalized to i  */

static inline v3_d v3d_add(v3_d a, v3_d b);    /* Pair-wise addition */
static inline v3_d v3d_adds(v3_d v, double s); /* Adds s to all fields */
static inline v3_d v3d_sub(v3_d a, v3_d b);    /* Pair-wise subtraction */
static inline v3_d v3d_subs(v3_d v, double s); /* Subs s to all fields */
static inline v3_d v3d_mul(v3_d a, v3_d b);    /* Pair-wise multiplication */
static inline v3_d v3d_muls(v3_d v, double s); /* Muls s to all fields */
static inline v3_d v3d_div(v3_d a, v3_d b);    /* Pair-wise division */
static inline v3_d v3d_divs(v3_d v, double s); /* Divs s to all fields */

static inline v3_d v3d_sum(int num, ...);                   /* Variatic addition */
static inline v3_d v3da_sum(size_t len, v3_d* array);       /* Sumation of an array */
static inline v3_d v3d_differnce(int num, ...);             /* Variatic subtraction */
static inline v3_d v3da_differnce(size_t len, v3_d* array); /* Difference of an array */
static inline v3_d v3d_product(int num, ...);               /* Variatic multiplication */
static inline v3_d v3da_product(size_t len, v3_d* array);   /* Product of an array */
static inline v3_d v3d_quotient(int num, ...);              /* Variatic division */
static inline v3_d v3da_quotient(size_t len, v3_d* array);  /* Quotient of an array */

static inline v3_d v3d_cross(v3_d a, v3_d b);          /* Cross-Product of a and b */
static inline v3_d v3d_proj(v3_d a, v3_d b);           /* Project a onto b */
static inline v3_d v3d_norm(v3_d v);                   /* Normalize v */
static inline v3_d v3d_fastnorm(v3_d v);               /* Fast Normalize. Same as norm but with some margin of error */
static inline v3_d v3d_lerp(v3_d a, v3_d b, double t); /* linear interp */

static inline double v3d_mag(v3_d v);              /* Magnitude of v */
static inline double v3d_fastmag(v3_d v);          /* Magnitude squared of v */
static inline double v3d_dist(v3_d a, v3_d b);     /* Magnitude of v3_sub(a, b) */
static inline double v3d_fastdist(v3_d a, v3_d b); /* Magnitude^2 of v3_sub(a, b) */
static inline double v3d_dot(v3_d a, v3_d b);      /* Dot Product of a and b */
static inline double v3d_angle(v3_d a, v3_d b);    /* Angle between a and b */
static inline int v3d_isZero(v3_d v);              /* Checks if v is zero */

// 4D Vector functions! These are just the
// decorations
// ---------------------------------------

static inline v4_d v4d(double x, double y, double z, double w); /* Returns a 4D Vector */
static inline v4_d v4dInit(double i); /* Returns vector initalized to i */

static inline v4_d v4d_add(v4_d a, v4_d b);    /* Pair-wise addition */
static inline v4_d v4d_adds(v4_d v, double s); /* Adds s to all fields */
static inline v4_d v4d_sub(v4_d a, v4_d b);    /* Pair-wise subtraction */
static inline v4_d v4d_subs(v4_d v, double s); /* Subs s to all fields */
static inline v4_d v4d_mul(v4_d a, v4_d b);    /* Pair-wise multiplication */
static inline v4_d v4d_muls(v4_d v, double s); /* Muls s to all fields */
static inline v4_d v4d_div(v4_d a, v4_d b);    /* Pair-wise division */
static inline v4_d v4d_divs(v4_d v, double s); /* Divs s to all fields */

static inline v4_d v4d_sum(int num, ...);                   /* Variatic addition */
static inline v4_d v4da_sum(size_t len, v4_d* array);       /* Sumation of an array */
static inline v4_d v4d_differnce(int num, ...);             /* Variatic subtraction */
static inline v4_d v4da_differnce(size_t len, v4_d* array); /* Difference of an array */
static inline v4_d v4d_product(int num, ...);               /* Variatic multiplication */
static inline v4_d v4da_product(size_t len, v4_d* array);   /* Product of an array */
static inline v4_d v4d_quotient(int num, ...);              /* Variatic division */
static inline v4_d v4da_quotient(size_t len, v4_d* array);  /* Quotient of an array */

static inline v4_d v4d_proj(v4_d a, v4_d b);           /* Project a onto b */
static inline v4_d v4d_norm(v4_d v);                   /* Normalize v */
static inline v4_d v4d_fastnorm(v4_d v);               /* Fast Normalize. Same as norm but with some margin of error */
static inline v4_d v4d_normAxis(v4_d v);               /* Normalize only v.axis */
static inline v4_d v4d_fastnormAxis(v4_d v);           /* Fast Normalize. Same as normAxis but with some margin of error */
static inline v4_d v4d_lerp(v4_d a, v4_d b, double t); /* linear interp */

static inline double v4d_mag(v4_d v);              /* Magnitude of v */
static inline double v4d_fastmag(v4_d v);          /* Magnitude squared of v */
static inline double v4d_dist(v4_d a, v4_d b);     /* Magnitude of v4_sub(a, b) */
static inline double v4d_fastdist(v4_d a, v4_d b); /* Magnitude^2 of v4_sub(a, b) */
static inline double v4d_dot(v4_d a, v4_d b);      /* Dot product of a and b */
static inline double v4d_angle(v4_d a, v4_d b);    /* Angle between a and b */
static inline int v4d_isZero(v4_d v);              /* Checks if v is zero */

// Quaternion functions! These are just the
// decorations
// ---------------------------------------

static inline qt_d qtd(double r, double i, double j, double k); /* Returns a quaternion */
static inline qt_d qtdInit(double i); /* Returns quaternion initalized to i */

static inline qt_d qtd_identity(void); /* Returns 1 + 0i + 0j + 0k */
static inline qt_d qtd_axis(double x, double y, double z, double angle); /* Axis-Angle to quaternion */
static inline qt_d qtdv_axis(v4_d axis); /* Axis angle to quaternion */

static inline qt_d qtd_add(qt_d a, qt_d b);    /* Pair-Wise addtion */
static inline qt_d qtd_adds(qt_d q, double s); /* Adds s to all fields */
static inline qt_d qtd_sub(qt_d a, qt_d b);    /* Pair-Wise subtraction */
static inline qt_d qtd_subs(qt_d q, double s); /* Subs s to all fields */
static inline qt_d qtd_mul(qt_d a, qt_d b);    /* Quaternion multiplication */
static inline qt_d qtd_muls(qt_d q, double s); /* Muls s to all fields */
static inline qt_d qtd_divs(qt_d q, double s); /* Divs s to all fields */

static inline qt_d qtd_sum(int num, ...);                   /* Variatic addition */
static inline qt_d qtda_sum(size_t len, qt_d* array);       /* Sumation of an array */
static inline qt_d qtd_differnce(int num, ...);             /* Variatic subtraction */
static inline qt_d qtda_differnce(size_t len, qt_d* array); /* Difference of an array */
static inline qt_d qtd_product(int num, ...);               /* Variatic multiplication */
static inline qt_d qtda_product(size_t len, qt_d* array);   /* Product of an array */

static inline qt_d qtd_sqrt(qt_d q);           /* Square-Root of a quaternion */
static inline qt_d qtd_exp(qt_d q);            /* Exponential of a quaternion */
static inline qt_d qtd_ln(qt_d q);             /* Natural Log of a quaternion */
static inline qt_d qtd_pows(qt_d q, double s); /* Quaternion to the power of s */

static inline qt_d qtd_conjugate(qt_d q);     /* q* = q.r - q.i - q.j - q.k */
static inline qt_d qtd_inverse(qt_d q);       /* q^-1 = q* / qt_fastmag(q) */
static inline qt_d qtd_cross(qt_d a, qt_d b); /* Cross product of a quaternion */
static inline qt_d qtd_norm(qt_d q);          /* Normalize q */
static inline qt_d qtd_fastnorm(qt_d q);      /* Fast Normalize. Same as norm but with some margin of error */
static inline qt_d qtd_lerp(qt_d a, qt_d b, double t);  /* linear interp */
static inline qt_d qtd_slerp(qt_d a, qt_d b, double t); /* spherical interp */

static inline double qtd_mag(qt_d q);             /* Magnitude of q */
static inline double qtd_fastmag(qt_d q);         /* Magnitude squared of q */
static inline double qtd_geonorm(qt_d a, qt_d b); /* Geodesic Distance, i.e., angle between */
static inline double qtd_dot(qt_d a, qt_d b);     /* Dot product */
static inline int qtd_isZero(qt_d q);             /* Checks if q is zero */
static inline int qtd_isUnit(qt_d q);             /* Checks if q is a unit quaternion (mag == 1) */
static inline int qtd_isIdentity(qt_d q);         /* Checks if q == 1 + 0i + 0j + 0k */

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

static inline m4x4_d m4dv(v4_d c0, v4_d c1, v4_d c2, v4_d c3); /* Returns a 4x4 Matrix */
static inline m4x4_d m4d(
        double x0, double x1, double x2, double x3,
        double y0, double y1, double y2, double y3,
        double z0, double z1, double z2, double z3,
        double w0, double w1, double w2, double w3); /* Returns a 4x4 Matrix */
static inline m4x4_d m4dInit(double i); /* Returns a 4x4 Matrix initalized to i */

static inline m4x4_d m4d_identity(void);                            /* Returns identity matrix */
static inline m4x4_d m4d_translation(double x, double y, double z); /* Returns translation matrix */
static inline m4x4_d m4dv_translation(v3_d offset);                 /* Returns translation matrix */
static inline m4x4_d m4d_scaling(double x, double y, double z);     /* Returns scaling matrix */
static inline m4x4_d m4dv_scaling(v3_d scale);                      /* Returns scaling matrix */

static inline m4x4_d m4d_xrot(double x); /* Returns rotation matrix for x-axis */
static inline m4x4_d m4d_yrot(double y); /* Returns rotation matrix for y-axis */
static inline m4x4_d m4d_zrot(double z); /* Returns rotation matrix for z-axis */
static inline m4x4_d m4d_perspective(
        double fovy, double aratio,
        double near_plane, double far_plane); /* Returns perspective matrix */
static inline m4x4_d m4d_ortho(
        double left, double right,
        double bottom, double top,
        double back, double front); /* Returns ortho matrix */

// Euler Rotation in order of X-Axis -> Y-Axis -> Z-Axis
static inline m4x4_d m4d_eulerXYZ(double x, double y, double z);
// Euler Rotation in order of X-Axis -> Y-Axis -> Z-Axis
static inline m4x4_d m4dv_eulerXYZ(v3_d angle);
// Euler Rotation in order of Z-Axis -> Y-Axis -> X-Axis
static inline m4x4_d m4d_eulerZYX(double z, double y, double x);
// Euler Rotation in order of Z-Axis -> Y-Axis -> X-Axis
static inline m4x4_d m4dv_eulerZYX(v3_d angle);

static inline m4x4_d m4d_axis(double x, double y, double z, double angle); /* Axis-Angle to matrix */
static inline m4x4_d m4dv_axis(v4_d axis); /* Axis-Angle to matrix */

static inline m4x4_d m4d_quaternion(double r, double i, double j, double k); /* Quaternion to matrix */
static inline m4x4_d m4dv_quaternion(qt_d q); /* Quaternion to matrix */

static inline m4x4_d m4d_transpose(m4x4_d m); /* Switch between column and row major order */
static inline m4x4_d m4d_add(m4x4_d a, m4x4_d b);  /* Pair-Wise addition */
static inline m4x4_d m4d_adds(m4x4_d m, double s); /* Adds s to all fields */
static inline m4x4_d m4d_sub(m4x4_d a, m4x4_d b);  /* Pair-Wise subtraction */
static inline m4x4_d m4d_subs(m4x4_d m, double s); /* Subs s to all fields */
static inline m4x4_d m4d_mul(m4x4_d a, m4x4_d b);  /* Matrix multiplication */
static inline m4x4_d m4d_muls(m4x4_d m, double s); /* Muls s to all fields */
static inline m4x4_d m4d_divs(m4x4_d m, double s); /* Divs s to all fields */

static inline m4x4_d m4d_sum(int num, ...);                     /* Variatic addition */
static inline m4x4_d m4da_sum(size_t len, m4x4_d* array);       /* Sumation of an array */
static inline m4x4_d m4d_differnce(int num, ...);               /* Variatic subtraction */
static inline m4x4_d m4da_differnce(size_t len, m4x4_d* array); /* Difference of an array */
static inline m4x4_d m4d_product(int num, ...);                 /* Variatic multiplication */
static inline m4x4_d m4da_product(size_t len, m4x4_d* array);   /* Product of an array */

// This is mostly a joke function. Like, why lerp a matrix?
static inline m4x4_d m4d_lerp(m4x4_d a, m4x4_d b, double t);

static inline v4_d m4d_mulv2(m4x4_d m, v2_d v); /* new_v = M*v */
static inline v4_d m4d_mulv3(m4x4_d m, v3_d v); /* new_v = M*v */
static inline v4_d m4d_mulv4(m4x4_d m, v4_d v); /* new_v = M*v */

static inline v4_d m4d_v2mul(v2_d v, m4x4_d m); /* new_v = v*M */
static inline v4_d m4d_v3mul(v3_d v, m4x4_d m); /* new_v = v*M */
static inline v4_d m4d_v4mul(v4_d v, m4x4_d m); /* new_v = v*M */

#ifdef LIN_DOUBLE_IMPLEMENTATION


// Helper Function Implementaions
// ------------------------------

static inline double fast_rsqrt(double number) {
    double y = number;
    double x2 = y * 0.5;
    long long i = *(long long*) &y;
    // The magic number is for doubles is from https://cs.uwaterloo.ca/~m32rober/rsqrt.pdf
    i = 0x5fe6eb50c7b537a9 - (i >> 1);
    y = *(double *) &i;
    y = y * (1.5 - (x2 * y * y));   // 1st iteration
    // y  = y * ( 1.5 - ( x2 * y * y ) );   // 2nd iteration, this can be removed
    return y;
}

static inline double lerp(double a, double b, double t) {
    return a * (1.0 - t) * b * t;
}


// 2D Vectors Implementaions
// -------------------------

static inline v2_d vec2d(double x, double y) {
    return (v2_d){ .x=x, .y=y };
}

static inline v2_d vec2dInit(double i) {
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


static inline v2_d v2d_sum(int num, ...) {
    va_list args;
    va_start(args, num);

    v2_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = v2d_add(result, va_arg(args, v2_d));
    }

    va_end(args);
    return result;
}

static inline v2_d v2da_sum(size_t len, v2_d* array) {
    v2_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v2d_add(result, *array);
    }

    return result;
}

static inline v2_d v2d_differnce(int num, ...) {
    va_list args;
    va_start(args, num);

    v2_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = v2d_sub(result, va_arg(args, v2_d));
    }

    va_end(args);
    return result;
}

static inline v2_d v2da_differnce(size_t len, v2_d* array) {
    v2_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v2d_sub(result, *array);
    }

    return result;
}

static inline v2_d v2d_product(int num, ...) {
    va_list args;
    va_start(args, num);

    v2_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = v2d_mul(result, va_arg(args, v2_d));
    }

    va_end(args);
    return result;
}

static inline v2_d v2da_product(size_t len, v2_d* array) {
    v2_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v2d_mul(result, *array);
    }

    return result;
}

static inline v2_d v2d_quotient(int num, ...) {
    va_list args;
    va_start(args, num);

    v2_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = v2d_div(result, va_arg(args, v2_d));
    }

    va_end(args);
    return result;
}

static inline v2_d v2da_quotient(size_t len, v2_d* array) {
    v2_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v2d_div(result, *array);
    }

    return result;
}


static inline v2_d v2d_proj(v2_d a, v2_d b) {
    double s = v2d_dot(a, b) / v2d_dot(b, b);
    return v2d_muls(b, s);
}

static inline v2_d v2d_norm(v2_d v) {
    return v2d_divs(v, v2d_mag(v));
}

static inline v2_d v2d_fastnorm(v2_d v) {
    double fast_mag = fast_rsqrt(v.x * v.x + v.y * v.y);
    return (v2_d){
        .x = v.x * fast_mag,
        .y = v.y * fast_mag,
    };
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

static inline double v2d_dist(v2_d a, v2_d b) {
    return v2d_mag(v2d_sub(a, b));
}

static inline double v2d_fastdist(v2_d a, v2_d b) {
    return v2d_fastmag(v2d_sub(a, b));
}

static inline double v2d_dot(v2_d a, v2_d b) {
    return a.x * b.x + a.y * b.y;
}

static inline double v2d_angle(v2_d a, v2_d b) {
    return acos(v2d_dot(a, b) / (v2d_mag(a) * v2d_mag(b)));
}

static inline int v2d_isZero(v2_d v) {
    return v2d_fastmag(v) < LIN_EPSILON;
}


// 3D Vectors Implementaions
// -------------------------

static inline v3_d v3d(double x, double y, double z) {
    return (v3_d){ .x=x, .y=y, .z=z };
}

static inline v3_d v3dInit(double i) {
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


static inline v3_d v3d_sum(int num, ...) {
    va_list args;
    va_start(args, num);

    v3_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = v3d_add(result, va_arg(args, v3_d));
    }

    va_end(args);
    return result;
}

static inline v3_d v3da_sum(size_t len, v3_d* array) {
    v3_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v3d_add(result, *array);
    }

    return result;
}

static inline v3_d v3d_differnce(int num, ...) {
    va_list args;
    va_start(args, num);

    v3_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = v3d_sub(result, va_arg(args, v3_d));
    }

    va_end(args);
    return result;
}

static inline v3_d v3da_differnce(size_t len, v3_d* array) {
    v3_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v3d_sub(result, *array);
    }

    return result;
}

static inline v3_d v3d_product(int num, ...) {
    va_list args;
    va_start(args, num);

    v3_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = v3d_mul(result, va_arg(args, v3_d));
    }

    va_end(args);
    return result;
}

static inline v3_d v3da_product(size_t len, v3_d* array) {
    v3_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v3d_mul(result, *array);
    }

    return result;
}

static inline v3_d v3d_quotient(int num, ...) {
    va_list args;
    va_start(args, num);

    v3_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = v3d_div(result, va_arg(args, v3_d));
    }

    va_end(args);
    return result;
}

static inline v3_d v3da_quotient(size_t len, v3_d* array) {
    v3_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v3d_div(result, *array);
    }

    return result;
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

static inline v3_d v3d_fastnorm(v3_d v) {
    double fast_mag = fast_rsqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return (v3_d){
        .x = v.x * fast_mag,
        .y = v.y * fast_mag,
        .z = v.z * fast_mag,
    };
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

static inline double v3d_dist(v3_d a, v3_d b) {
    return v3d_mag(v3d_sub(a, b));
}

static inline double v3d_fastdist(v3_d a, v3_d b) {
    return v3d_fastmag(v3d_sub(a, b));
}

static inline double v3d_dot(v3_d a, v3_d b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline double v3d_angle(v3_d a, v3_d b) {
    return acos(v3d_dot(a, b) / (v3d_mag(a) * v3d_mag(b)));
}

static inline int v3d_isZero(v3_d v) {
    return v3d_fastmag(v) < LIN_EPSILON;
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


static inline v4_d v4d_sum(int num, ...) {
    va_list args;
    va_start(args, num);

    v4_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = v4d_add(result, va_arg(args, v4_d));
    }

    va_end(args);
    return result;
}

static inline v4_d v4da_sum(size_t len, v4_d* array) {
    v4_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v4d_add(result, *array);
    }

    return result;
}

static inline v4_d v4d_differnce(int num, ...) {
    va_list args;
    va_start(args, num);

    v4_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = v4d_sub(result, va_arg(args, v4_d));
    }

    va_end(args);
    return result;
}

static inline v4_d v4da_differnce(size_t len, v4_d* array) {
    v4_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v4d_sub(result, *array);
    }

    return result;
}

static inline v4_d v4d_product(int num, ...) {
    va_list args;
    va_start(args, num);

    v4_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = v4d_mul(result, va_arg(args, v4_d));
    }

    va_end(args);
    return result;
}

static inline v4_d v4da_product(size_t len, v4_d* array) {
    v4_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v4d_mul(result, *array);
    }

    return result;
}

static inline v4_d v4d_quotient(int num, ...) {
    va_list args;
    va_start(args, num);

    v4_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = v4d_div(result, va_arg(args, v4_d));
    }

    va_end(args);
    return result;
}

static inline v4_d v4da_quotient(size_t len, v4_d* array) {
    v4_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = v4d_div(result, *array);
    }

    return result;
}


static inline v4_d v4d_proj(v4_d a, v4_d b) {
    double s = v4d_dot(a, b) / v4d_dot(b, b);
    return v4d_muls(b, s);
}

static inline v4_d v4d_norm(v4_d v) {
    return v4d_divs(v, v4d_mag(v));
}

static inline v4_d v4d_fastnorm(v4_d v) {
    double fast_mag = fast_rsqrt(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
    return (v4_d){
        .x = v.x * fast_mag,
        .y = v.y * fast_mag,
        .z = v.z * fast_mag,
        .w = v.w * fast_mag,
    };
}

static inline v4_d v4d_normAxis(v4_d v) {
    return (v4_d){
        .axis = v3d_norm(v.axis),
        .angle = v.angle,
    };
}

static inline v4_d v4d_fastnormAxis(v4_d v) {
    double fast_mag = fast_rsqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return (v4_d){
        .x = v.x * fast_mag,
        .y = v.y * fast_mag,
        .z = v.z * fast_mag,
        .angle = v.angle,
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

static inline double v4d_dist(v4_d a, v4_d b) {
    return v4d_mag(v4d_sub(a, b));
}

static inline double v4d_fastdist(v4_d a, v4_d b) {
    return v4d_fastmag(v4d_sub(a, b));
}

static inline double v4d_dot(v4_d a, v4_d b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static inline double v4d_angle(v4_d a, v4_d b) {
    return acos(v4d_dot(a, b) / (v4d_mag(a) * v4d_mag(b)));
}

static inline int v4d_isZero(v4_d v) {
    return v4d_fastmag(v) < LIN_EPSILON;
}


// Quaternion Implementaions
// -------------------------

static inline qt_d qtd(double r, double i, double j, double k) {
    return (qt_d){ .r = r, .i = i, .j = j, .k = k };
}

static inline qt_d qtdInit(double i) {
    return (qt_d){ .r = i, .i = i, .j = i, .k = i };
}


static inline qt_d qtd_identity(void) {
    return (qt_d){ .r = 1.0, .i = 0.0, .j = 0.0, .k = 0.0 };
}

static inline qt_d qtd_axis(double x, double y, double z, double angle) {
    double sna = sin(angle);
    return (qt_d){
        .r = cos(angle),
        .i = x * sna,
        .j = y * sna,
        .k = z * sna,
    };
}

static inline qt_d qtdv_axis(v4_d axis) {
    double sna = sin(axis.angle);
    return (qt_d){
        .r = cos(axis.angle),
        .i = axis.x * sna,
        .j = axis.y * sna,
        .k = axis.z * sna,
    };
}


static inline qt_d qtd_add(qt_d a, qt_d b) {
    return (qt_d){
        .r = a.r + b.r,
        .i = a.i + b.i,
        .j = a.j + b.j,
        .k = a.k + b.k,
    };
}

static inline qt_d qtd_adds(qt_d q, double s) {
    return (qt_d){
        .r = q.r + s,
        .i = q.i + s,
        .j = q.j + s,
        .k = q.k + s,
    };
}

static inline qt_d qtd_sub(qt_d a, qt_d b) {
    return (qt_d){
        .r = a.r - b.r,
        .i = a.i - b.i,
        .j = a.j - b.j,
        .k = a.k - b.k,
    };
}

static inline qt_d qtd_subs(qt_d q, double s) {
    return (qt_d){
        .r = q.r - s,
        .i = q.i - s,
        .j = q.j - s,
        .k = q.k - s,
    };
}

static inline qt_d qtd_mul(qt_d a, qt_d b) {
    double r = (a.r * b.r) - (a.i * b.i) - (a.j * b.j) - (a.k * b.k);
    double i = (a.r * b.i) + (a.i * b.r) - (a.j * b.k) + (a.k * b.j);
    double j = (a.r * b.j) + (a.i * b.k) + (a.j * b.r) - (a.k * b.i);
    double k = (a.r * b.k) - (a.i * b.j) + (a.j * b.i) + (a.k * b.r);
    
    return (qt_d){ .r = r, .i = i, .j = j, .k = k };
}

static inline qt_d qtd_muls(qt_d q, double s) {
    return (qt_d){
        .r = q.r * s,
        .i = q.i * s,
        .j = q.j * s,
        .k = q.k * s,
    };
}

static inline qt_d qtd_divs(qt_d q, double s) {
    return (qt_d){
        .r = q.r / s,
        .i = q.i / s,
        .j = q.j / s,
        .k = q.k / s,
    };
}


static inline qt_d qtd_sum(int num, ...) {
    va_list args;
    va_start(args, num);

    qt_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = qtd_add(result, va_arg(args, qt_d));
    }

    va_end(args);
    return result;
}

static inline qt_d qtda_sum(size_t len, qt_d* array) {
    qt_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = qtd_add(result, *array);
    }

    return result;
}

static inline qt_d qtd_differnce(int num, ...) {
    va_list args;
    va_start(args, num);

    qt_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = qtd_sub(result, va_arg(args, qt_d));
    }

    va_end(args);
    return result;
}

static inline qt_d qtda_differnce(size_t len, qt_d* array) {
    qt_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = qtd_sub(result, *array);
    }

    return result;
}

static inline qt_d qtd_product(int num, ...) {
    va_list args;
    va_start(args, num);

    qt_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = qtd_mul(result, va_arg(args, qt_d));
    }

    va_end(args);
    return result;
}

static inline qt_d qtda_product(size_t len, qt_d* array) {
    qt_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = qtd_mul(result, *array);
    }

    return result;
}

static inline qt_d qtd_sqrt(qt_d q) {
    double qmag = sqrt(q.r * q.r + q.i * q.i + q.j * q.j + q.k * q.k);

    double r = sqrt((qmag + q.r) / 2.0);
    v3_d v = v3d_muls(v3d_norm(q.v), (qmag - q.r) / 2.0);
    return (qt_d){ .w = r, .v = v };
}

static inline qt_d qtd_exp(qt_d q) {
    double vmag = v3d_mag(q.v);
    
    double r = pow(LIN_E_F, q.r) * cos(vmag);
    v3_d v = v3d_muls(v3d_divs(q.v, vmag), sin(vmag));
    return (qt_d){ .w = r, .v = v };
}

static inline qt_d qtd_ln(qt_d q) {
    double qmag = sqrt(q.r * q.r + q.i * q.i + q.j * q.j + q.k * q.k);

    double r = log(qmag);
    v3_d v = v3d_muls(v3d_norm(q.v), acos(q.r / qmag));
    return (qt_d){ .w = r, .v = v };
}

static inline qt_d qtd_pows(qt_d q, double s) {
    return qtd_exp(qtd_muls(qtd_ln(q), s));
}


static inline qt_d qtd_conjugate(qt_d q) {
    return (qt_d){ .r = q.r, .i = -q.i, .j = -q.j, .k = -q.k };
}

static inline qt_d qtd_inverse(qt_d q) {
    double qmag = q.r * q.r + q.i * q.i + q.j * q.j + q.k * q.k;
    return (qt_d){
        .r=( q.r / qmag),
        .i=(-q.i / qmag),
        .j=(-q.j / qmag),
        .k=(-q.k / qmag),
    };
}

static inline qt_d qtd_cross(qt_d a, qt_d b) {
    return (qt_d){
        .r = 0.0,
        .i = a.j * b.k - a.k * b.j,
        .j = a.k * b.j - a.i * b.k,
        .k = a.i * b.j - a.j * b.i,
    };
}

static inline qt_d qtd_norm(qt_d q) {
    double qmag = sqrt(q.r * q.r + q.i * q.i + q.j * q.j + q.k * q.k);
    return (qt_d){
        .r = q.r / qmag,
        .i = q.i / qmag,
        .j = q.j / qmag,
        .k = q.k / qmag,
    };
}

static inline qt_d qtd_fastnorm(qt_d q) {
    double fast_qmag = fast_rsqrt(q.r * q.r + q.i * q.i + q.j * q.j + q.k * q.k);
    return (qt_d){
        .r = q.r * fast_qmag,
        .i = q.i * fast_qmag,
        .j = q.j * fast_qmag,
        .k = q.k * fast_qmag,
    };
}

static inline qt_d qtd_lerp(qt_d a, qt_d b, double t) {
    return qtd_add(qtd_muls(a, 1.0 - t), qtd_muls(b, t));
}

static inline qt_d qtd_slerp(qt_d a, qt_d b, double t) {
    return qtd_mul(a, qtd_pows(qtd_mul(qtd_inverse(a), b), t));
}


static inline double qtd_mag(qt_d q) {
    return sqrt(q.r * q.r + q.i * q.i + q.j * q.j + q.k * q.k);
}

static inline double qtd_fastmag(qt_d q) {
    return q.r * q.r + q.i * q.i + q.j * q.j + q.k * q.k;
}

static inline double qtd_geonorm(qt_d a, qt_d b) {
    return qtd_mag(qtd_ln(qtd_mul(qtd_inverse(a), b)));
}

static inline double qtd_dot(qt_d a, qt_d b) {
    return a.r * b.r + a.i * b.i + a.j * b.j + a.k * b.k;
}

static inline int qtd_isZero(qt_d q) {
    return qtd_fastmag(q) < LIN_EPSILON;
}

static inline int qtd_isUnit(qt_d q) {
    return (qtd_fastmag(q) - 1.0) < LIN_EPSILON;
}

static inline int qtd_isIdentity(qt_d q) {
    return (q.r - 1.0) < LIN_EPSILON && v3d_isZero(q.v);
}


// Vector Casting Implementaions
// -----------------------------

static inline v2_d v3d_to_v2d(v3_d v) {
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

static inline m4x4_d m4d_translation(double x, double y, double z) {
    return (m4x4_d){
        .x0=1.0, .y0=0.0, .z0=0.0, .w0=0.0,
        .x1=0.0, .y1=1.0, .z1=0.0, .w1=0.0,
        .x2=0.0, .y2=0.0, .z2=1.0, .w2=0.0,
        .x3=x,   .y3=y,   .z3=z,   .w3=1.0,
    };
}

static inline m4x4_d m4dv_translation(v3_d offset) {
    return (m4x4_d){
        .x0=1.0,      .y0=0.0,      .z0=0.0,      .w0=0.0,
        .x1=0.0,      .y1=1.0,      .z1=0.0,      .w1=0.0,
        .x2=0.0,      .y2=0.0,      .z2=1.0,      .w2=0.0,
        .x3=offset.x, .y3=offset.y, .z3=offset.z, .w3=1.0,
    };
}

static inline m4x4_d m4d_scaling(double x, double y, double z) {
    return (m4x4_d){
        .x0=x,   .y0=0.0, .z0=0.0, .w0=0.0,
        .x1=0.0, .y1=y,   .z1=0.0, .w1=0.0,
        .x2=0.0, .y2=0.0, .z2=z,   .w2=0.0,
        .x3=0.0, .y3=0.0, .z3=0.0, .w3=1.0,
    };
}

static inline m4x4_d m4dv_scaling(v3_d scale) {
    return (m4x4_d){
        .x0=scale.x, .y0=0.0,     .z0=0.0,     .w0=0.0,
        .x1=0.0,     .y1=scale.y, .z1=0.0,     .w1=0.0,
        .x2=0.0,     .y2=0.0,     .z2=scale.z, .w2=0.0,
        .x3=0.0,     .y3=0.0,     .z3=0.0,     .w3=1.0,
    };
}


static inline m4x4_d m4d_xrot(double x) {
    double snx = sin(x);
    double csx = cos(x);

    return (m4x4_d){
        .x0=1.0, .x1=0.0, .x2= 0.0, .x3=0.0,
        .y0=0.0, .y1=csx, .y2=-snx, .y3=0.0,
        .z0=0.0, .z1=snx, .z2= csx, .z3=0.0,
        .w0=0.0, .w1=0.0, .w2= 0.0, .w3=1.0,
    };
}

static inline m4x4_d m4d_yrot(double y) {
    double sny = sin(y);
    double csy = cos(y);

    return (m4x4_d){
        .x0= csy, .x1=0.0, .x2=sny, .x3=0.0,
        .y0= 0.0, .y1=1.0, .y2=0.0, .y3=0.0,
        .z0=-sny, .z1=0.0, .z2=csy, .z3=0.0,
        .w0= 0.0, .w1=0.0, .w2=0.0, .w3=1.0,
    };
}

static inline m4x4_d m4d_zrot(double z) {
    double snz = sin(z);
    double csz = cos(z);

    return (m4x4_d){
        .x0=csz, .x1=-snz, .x2=0.0, .x3=0.0,
        .y0=snz, .y1= csz, .y2=0.0, .y3=0.0,
        .z0=0.0, .z1= 0.0, .z2=1.0, .z3=0.0,
        .w0=0.0, .w1= 0.0, .w2=0.0, .w3=1.0,
    };
}

static inline m4x4_d m4d_perspective(double fovy, double aratio, double near_plane, double far_plane) {
    double f = 1.0 / tan(fovy / 2.0);

    m4x4_d result = m4d_scaling(
        f / aratio,
        f,
        (far_plane + near_plane) / (near_plane - far_plane)
    );

    result.z3 = (2.0 * far_plane * near_plane) / (near_plane - far_plane);
    result.w2 = -1.0;

    return result;
}

static inline m4x4_d m4d_ortho(
        double left, double right,
        double bottom, double top,
        double back, double front) {

    m4x4_d result = m4d_scaling(
            2.0 / (right - left),
            2.0 / (top - bottom),
            2.0 / (back - front)
    );

    result.x3 = -(right + left) / (right - left);
    result.y3 = -(top + bottom) / (top - bottom);
    result.z3 = -(back + front) / (back - front);

    return result;
}


// Euler Rotation in order of X-Axis -> Y-Axis -> Z-Axis
static inline m4x4_d m4d_eulerXYZ(double x, double y, double z) {
    double snx = sin(-x), sny = sin(-y), snz = sin(-z);
    double csx = cos(-x), csy = cos(-y), csz = cos(-z);

    v4_d c0 = {
	    .x =  csy * csz,
	    .y = -csx * snz + snx * sny * csz,
	    .z =  snx * snz + csx * sny * csz,
	    .w =  0.0,
    };

    v4_d c1 = {
	    .x =  csy * snz,
	    .y =  csx * csz + snx * sny * snz,
	    .z = -snx * csz + csx * sny * snz,
	    .w =  0.0,
    };

    v4_d c2 = {
	    .x = -sny,
	    .y =  snx * csy,
	    .z =  csx * csy,
	    .w =  0.0,
    };

    v4_d c3 = { .x = 0.0, .y = 0.0, .z = 0.0, .w = 1.0 };
    return (m4x4_d){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}

// Euler Rotation in order of X-Axis -> Y-Axis -> Z-Axis
static inline m4x4_d m4dv_eulerXYZ(v3_d angle) {
    double snx = sin(-angle.x), sny = sin(-angle.y), snz = sin(-angle.z);
    double csx = cos(-angle.x), csy = cos(-angle.y), csz = cos(-angle.z);

    v4_d c0 = {
	    .x =  csy * csz,
	    .y = -csx * snz + snx * sny * csz,
	    .z =  snx * snz + csx * sny * csz,
        .w =  0.0,
    };

    v4_d c1 = {
	    .x =  csy * snz,
	    .y =  csx * csz + snx * sny * snz,
	    .z = -snx * csz + csx * sny * snz,
        .w =  0.0,
    };

    v4_d c2 = {
	    .x = -sny,
	    .y =  snx * csy,
	    .z =  csx * csy,
        .w =  0.0,
    };

    v4_d c3 = { .x = 0.0, .y = 0.0, .z = 0.0, .w = 1.0 };
    return (m4x4_d){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}

// Euler Rotation in order of Z-Axis -> Y-Axis -> X-Axis
static inline m4x4_d m4d_eulerZYX(double z, double y, double x) {
    double snx = sin(-x), sny = sin(-y), snz = sin(-z);
    double csx = cos(-x), csy = cos(-y), csz = cos(-z);

    v4_d c0 = {
        .x = csz * csy,
	    .y = csy * snz,
	    .z =-sny,
        .w = 0.0,
    };

    v4_d c1 = {
	    .x = csz * sny * snx - csx * snz,
	    .y = csz * csx + snz * sny * snx,
	    .z = csy * snx,
        .w = 0.0,
    };

    v4_d c2 = {
	    .x = snz * snx + csz * csx * sny,
	    .y = csx * snz * sny - csz * snx,
	    .z = csy * csx,
        .w = 0.0,
    };
    

    v4_d c3 = { .x = 0.0, .y = 0.0, .z = 0.0, .w = 1.0 };
    return (m4x4_d){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}

// Euler Rotation in order of Z-Axis -> Y-Axis -> X-Axis
static inline m4x4_d m4dv_eulerZYX(v3_d angle) {
    double snx = sin(-angle.x), sny = sin(-angle.y), snz = sin(-angle.z);
    double csx = cos(-angle.x), csy = cos(-angle.y), csz = cos(-angle.z);

    v4_d c0 = {
        .x = csz * csy,
	    .y = csy * snz,
	    .z =-sny,
        .w = 0.0,
    };

    v4_d c1 = {
	    .x = csz * sny * snx - csx * snz,
	    .y = csz * csx + snz * sny * snx,
	    .z = csy * snx,
        .w = 0.0,
    };

    v4_d c2 = {
	    .x = snz * snx + csz * csx * sny,
	    .y = csx * snz * sny - csz * snx,
	    .z = csy * csx,
        .w = 0.0,
    };
    

    v4_d c3 = { .x = 0.0, .y = 0.0, .z = 0.0, .w = 1.0 };
    return (m4x4_d){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}


static inline m4x4_d m4d_axis(double x, double y, double z, double angle) {
    double csa = cos(angle); double sna = sin(angle);
    v4_d c0 = (v4_d){
        .x = csa + (x * x) * (1.0 - csa),
        .y = y * x * (1.0 - csa) + z * sna,
        .z = z * x * (1.0 - csa) - y * sna,
	};

    v4_d c1 = (v4_d){
        .x = x * y * (1.0 - csa) - z * sna,
        .y = csa + (y * y) * (1.0 - csa),
        .z = z * y * (1.0 - csa) + x * sna,
	};

    v4_d c2 = (v4_d){
        .x = x * z * (1.0 - csa) + y * sna,
        .y = y * z * (1.0 - csa) - x * sna,
        .z = csa + (z * z) * (1.0 - csa),
	};


    v4_d c3 = { .x = 0.0, .y = 0.0, .z = 0.0, .w = 1.0 };
    return (m4x4_d){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}

static inline m4x4_d m4dv_axis(v4_d axis) {
    double csa = cos(axis.angle); double sna = sin(axis.angle);
    v4_d c0 = (v4_d){
        .x = csa + (axis.x * axis.x) * (1.0 - csa),
        .y = axis.y * axis.x * (1.0 - csa) + axis.z * sna,
        .z = axis.z * axis.x * (1.0 - csa) - axis.y * sna,
	};

    v4_d c1 = (v4_d){
        .x = axis.x * axis.y * (1.0 - csa) - axis.z * sna,
        .y = csa + (axis.y * axis.y) * (1.0 - csa),
        .z = axis.z * axis.y * (1.0 - csa) + axis.x * sna,
	};

    v4_d c2 = (v4_d){
        .x = axis.x * axis.z * (1.0 - csa) + axis.y * sna,
        .y = axis.y * axis.z * (1.0 - csa) - axis.x * sna,
        .z = csa + (axis.z * axis.z) * (1.0 - csa),
	};


    v4_d c3 = { .x = 0.0, .y = 0.0, .z = 0.0, .w = 1.0 };
    return (m4x4_d){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}


static inline m4x4_d m4d_quaternion(double r, double i, double j, double k) {
    v4_d c0 = {
        .x = 1.0 - 2.0 * (i * i + j * j),
        .y = 2.0 * (r * i + k * j),
        .z = 2.0 * (r * j - k * i),
        .w = 0.0,
    };

    v4_d c1 = {
        .x = 2.0 * (r * i - k * j),
        .y = 1.0 - 2.0 * (r * r + j * j),
        .z = 2.0 * (i * j + k * r),
        .w = 0.0,
    };

    v4_d c2 = {
        .x = 2.0 * (r * j + k * i),
        .y = 2.0 * (i * j - k * r),
        .z = 1.0 - 2.0 * (r * r + i * i),
        .w = 0.0,
    };

    v4_d c3 = { .x = 0.0, .y = 0.0, .z = 0.0, .w = 1.0 };
	return (m4x4_d){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}

static inline m4x4_d m4dv_quaternion(qt_d q) {
    v4_d c0 = {
        .x = 1.0 - 2.0 * (q.i * q.i + q.j * q.j),
        .y = 2.0 * (q.r * q.i + q.k * q.j),
        .z = 2.0 * (q.r * q.j - q.k * q.i),
        .w = 0.0,
    };

    v4_d c1 = {
        .x = 2.0 * (q.r * q.i - q.k * q.j),
        .y = 1.0 - 2.0 * (q.r * q.r + q.j * q.j),
        .z = 2.0 * (q.i * q.j + q.k * q.r),
        .w = 0.0,
    };

    v4_d c2 = {
        .x = 2.0 * (q.r * q.j + q.k * q.i),
        .y = 2.0 * (q.i * q.j - q.k * q.r),
        .z = 1.0 - 2.0 * (q.r * q.r + q.i * q.i),
        .w = 0.0,
    };

    v4_d c3 = { .x = 0.0, .y = 0.0, .z = 0.0, .w = 1.0 };
	return (m4x4_d){ .c0 = c0, .c1 = c1, .c2 = c2, .c3 = c3 };
}



static inline m4x4_d m4d_sum(int num, ...) {
    va_list args;
    va_start(args, num);

    m4x4_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = m4d_add(result, va_arg(args, m4x4_d));
    }

    va_end(args);
    return result;
}

static inline m4x4_d m4da_sum(size_t len, m4x4_d* array) {
    m4x4_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = m4d_add(result, *array);
    }

    return result;
}

static inline m4x4_d m4d_differnce(int num, ...) {
    va_list args;
    va_start(args, num);

    m4x4_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = m4d_sub(result, va_arg(args, m4x4_d));
    }

    va_end(args);
    return result;
}

static inline m4x4_d m4da_differnce(size_t len, m4x4_d* array) {
    m4x4_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = m4d_sub(result, *array);
    }

    return result;
}

static inline m4x4_d m4d_product(int num, ...) {
    va_list args;
    va_start(args, num);

    m4x4_d result = { 0 };
    for (int i = 0; i < num; ++i) {
        result = m4d_mul(result, va_arg(args, m4x4_d));
    }

    va_end(args);
    return result;
}

static inline m4x4_d m4da_product(size_t len, m4x4_d* array) {
    m4x4_d result = { 0 };
    void* end;
    for (end = &array[len-1]; array != end; ++array) {
        result = m4d_mul(result, *array);
    }

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

static inline m4x4_d m4d_add(m4x4_d a, m4x4_d b) {
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
        .x = a.x0*b.x3 + a.x1*b.y3 + a.x2*b.z3 + a.x3*b.w3,
        .y = a.y0*b.x3 + a.y1*b.y3 + a.y2*b.z3 + a.y3*b.w3,
        .z = a.z0*b.x3 + a.z1*b.y3 + a.z2*b.z3 + a.z3*b.w3,
        .w = a.w0*b.x3 + a.w1*b.y3 + a.w2*b.z3 + a.w3*b.w3,
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

static inline m4x4_d m4d_divs(m4x4_d m, double s) {
    return (m4x4_d){
        .x0=m.x0 / s, .y0=m.y0 / s, .z0=m.z0 / s, .w0=m.w0 / s,
        .x1=m.x1 / s, .y1=m.y1 / s, .z1=m.z1 / s, .w1=m.w1 / s,
        .x2=m.x2 / s, .y2=m.y2 / s, .z2=m.z2 / s, .w2=m.w2 / s,
        .x3=m.x3 / s, .y3=m.y3 / s, .z3=m.z3 / s, .w3=m.w3 / s,
    };
}

static inline m4x4_d m4d_lerp(m4x4_d a, m4x4_d b, double t) {
    return (m4x4_d) {
        .c0 = v4d_lerp(a.c0, b.c0, t),
        .c1 = v4d_lerp(a.c1, b.c1, t),
        .c2 = v4d_lerp(a.c2, b.c2, t),
        .c3 = v4d_lerp(a.c3, b.c3, t),
    };
}


static inline v4_d m4d_mulv2(m4x4_d m, v2_d v) {
    return (v4_d){
        .x = v.x * m.x0 + v.y * m.x1,
        .y = v.x * m.y0 + v.y * m.y1,
        .z = v.x * m.z0 + v.y * m.z1,
        .w = v.x * m.w0 + v.y * m.w1,
    };
}

static inline v4_d m4d_mulv3(m4x4_d m, v3_d v) {
    return (v4_d){
        .x = v.x * m.x0 + v.y * m.x1 + v.z * m.x2,
        .y = v.x * m.y0 + v.y * m.y1 + v.z * m.y2,
        .z = v.x * m.z0 + v.y * m.z1 + v.z * m.z2,
        .w = v.x * m.w0 + v.y * m.w1 + v.z * m.w2,
    };
}

static inline v4_d m4d_mulv4(m4x4_d m, v4_d v) {
    return (v4_d){
        .x = v.x * m.x0 + v.y * m.x1 + v.z * m.x2 + v.w * m.x3,
        .y = v.x * m.y0 + v.y * m.y1 + v.z * m.y2 + v.w * m.y3,
        .z = v.x * m.z0 + v.y * m.z1 + v.z * m.z2 + v.w * m.z3,
        .w = v.x * m.w0 + v.y * m.w1 + v.z * m.w2 + v.w * m.w3,
    };
}

static inline v4_d m4d_v2mul(v2_d v, m4x4_d m) {
	return (v4_d){
        .x = v.x * m.x0 + v.y * m.y0,
        .y = v.x * m.x1 + v.y * m.y1,
        .z = v.x * m.x2 + v.y * m.y2,
        .w = v.x * m.x3 + v.y * m.y3,
	};
}

static inline v4_d m4d_v3mul(v3_d v, m4x4_d m) {
	return (v4_d){
        .x = v.x * m.x0 + v.y * m.y0 + v.z * m.z0,
        .y = v.x * m.x1 + v.y * m.y1 + v.z * m.z1,
        .z = v.x * m.x2 + v.y * m.y2 + v.z * m.z2,
        .w = v.x * m.x3 + v.y * m.y3 + v.z * m.z3,
	};
}

static inline v4_d m4d_v4mul(v4_d v, m4x4_d m) {
	return (v4_d){
        .x = v.x * m.x0 + v.y * m.y0 + v.z * m.z0 + v.w * m.w0,
        .y = v.x * m.x1 + v.y * m.y1 + v.z * m.z1 + v.w * m.w1,
        .z = v.x * m.x2 + v.y * m.y2 + v.z * m.z2 + v.w * m.w2,
        .w = v.x * m.x3 + v.y * m.y3 + v.z * m.z3 + v.w * m.w3,
	};
}

#endif

#endif
