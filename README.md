
#  lin_math Linear Algebra Library

A Single-Header only C math library for linear algebra. Supports 2D, 3D 4D Vectors, 4 by 4 matrices, and quaternions.

Nearly all functions are both static and inlined. Functions are very small so boat shouldn't be a problem. (Might add a macro to disable inlining it all if it ends up being a problem idk)


## Importing the library

As a single-header only library by default importing the file will only include the function declarations and constants. It will also only import support for 32-bit floats. In only one of the .c files use the macro `#define LIN_MATH_IMPLEMENTATION`

This will enable the function definitions for the 32-bit float functions. The following are macros for enabling different features:

| Macro                       | Description                                       |
| :-------------------------- | :------------------------------------------------ |
| `LIN_MATH_IMPLEMENTATION`   | Enables 32-bit float function ***definitions***   |
| `LIN_DOUBLE`                | Enables 64-bit double function ***declarations*** |
| `LIN_DOUBLE_IMPLEMENTATION` | Enables 64-bit double function ***definitions***  |
| `LIN_ENABLE_ALL`            | Enables all of the above                          |

# Library Reference

## Types

| **Type** | **Double Variant** | **Description** |
| :------- | :-------------- | :----------------- |
| v2_t     | v2_d            | 2D Vector          |
| v3_t     | v3_d            | 3D Vector          |
| v4_t     | v4_d            | 4D Vector          |
| qt_t     | qt_d            | Quaternion         |
| m4x4_t   | m4x4_d          | 4 By 4 Matrix      |

All types are unions of anonymous structs. Which is a cool way of saying that they call all be accessed in mutliple ways. For example: A 4D Vector type can be accessed as `{ x y z w }` or `{ r g b a }` Both are accessing the same elements; just with different names. Vector types call also cast to lower dimensions if need be.

```c
// 4D Vectors
v4_t vec4 = { .x = 1.0f, .y = 6.0f, .z = 2.0f, .w = 8.0f };

// 3D Vectors
v3_t vec3 = { .x = 6.5f, .y = 1.1f, .z = 7.7f };
v3_t from_4d = vec4.v3;

// 2D Vectors
v2_t vec2 = { .x = 4.0f, .y = 5.0f };
v2_t from_4d = vec4.v2;
v2_t from_3d = vec3.v2;

// Quaternions
qt_t quat = { .r = 0.4f, .i = 0.1f, .j = 0.4f, .k = 9.9f };

qt_t from_v4 = qtv(vec4); /* this one is a function call */
v4_t from_qt = quat.v4;
```

Because all types are just union they can all be thought of as an array of N floats (or doubles). In memory a v2_t is just 2 floats. And a v4_t is 4 floats. A matrix is 16 floats.

In memory a matrix is stored in **Column-Major Order**. Personally I get confused about column-major and row-major all the time. I always somehow mix them up. If you are anything like me this might help. Within memory a given matrix is stored as an array of 4D vectors. Each 4D vector is a column. Here is something that will hopefully help explain better than words alone will:

```
As often seen on paper
    / x0  x1  x2  x3 \
M = | y0  y1  y2  y3 |
    | z0  z1  z2  z3 |
    \ w0  w1  w2  w3 /

The number next to the letter is the column!
The letter is the row! x is row0, y is row1, z is row2, w is row3

Inside memory
index    |  0  1  2  3   4  5  6  7   8  9  A  B   C  D  E  F
_________|___________________________________________________
Elements | x0 y0 z0 w0  x1 y1 z1 w1  x2 y2 z2 w2  x3 y3 z3 w3 
```

If one is accessing a matrix as a 2D array, then it should be done like so: `float value = matrix.m[colmn][row];` For examaple y2 would be `float y2 = matrix.m[3][1];` because the 2 is the third colmn, and y is always row 1.

OpenGL expects a matrix to be in column-major order. Thus there is no need to transpose the matrix before using it. For example in a OpenGL uniform they can be used as such `glUniformMatrix4fv(location, 1, GL_FALSE, &matrix.m[0][0]);` If the matrix has been transposed (e.g., with `m4_transpose`) then be sure to change `GL_FALSE` to `GL_TRUE` or to transpose the matrix again.

### 2D Vector <===> **v2_t**

| **Context** | **Fields**            |
| :---------- | :-------------------- |
| Vector      | `float x, y`          |
| Texture     | `float u, v`          |
| Color       | `float r, g`          |
| Imaginary   | `float real, i`       |
| Size        | `float width, height` |
| Raw Data    | `float data[2]`       |

### 3D Vector <===> **v3_t**
| **Context**  | **Fields**                   |
| :----------- | :--------------------------- |
| Vector       | `float x, y, z`              |
| Texture      | `float u, v, t`              |
| Color        | `float r, g, b`              |
| Imaginary    | `float real, i, j`           |
| Size         | `float width, height, depth` |
| Euler Angles | `float pitch, yaw, roll`     |
| Raw Data     | `float data[3]`              |
|              |                              |
| Cast Down    |  `v2_t v2`                   |

### 4D Vector <===> **v4_t**
| **Context** | **Fields**                         |
| :---------- | :--------------------------------- |
| Vector      | `float x, y, z, w`                 |
| Texture     | `float u, v, t, s`                 |
| Color       | `float r, g, b, a`                 |
| Imaginary   | `float real, i, j, k`              |
| Size        | `float width, height, depth, time` |
| Axis Angle  | `v3_t axis; float angle`           |
| Raw Data    | `float data[4]`                    |
|             |                                    |
| Cast Down   |  `v2_t v2`  `v3_t v3`              |

### Quaternion <===> **qt_t**
| **Context** | **Fields**         |
| :---------- | :----------------- |
| Quaternion  | `float r, i, j, k` |
| Vector Part | `float w; v3_t v`  |
| Raw Data    | `float data[4]`    |
|             |                    |
| Casting     |  `v4_t v4`         |

### 4x4 Matrix <===> **m4x4_t**
| **Context** | **Fields**            |
| :---------- | :---------------------|
| Columns     | `v4_t c0, c1, c2, c3` |
| 2D Array    | `float m[4][4]`       |
| Raw Data    | `float data[16]`      |
|             |                       |
| Elements    | `float x0, x1, x2, x3;` <br> `float y0, y1, y2, y3;` <br> `float z0, z1, z2, z3;` <br> `float w0, w1, w2, w3;`|

# Functions

All functions are prefixed with the type they operate on (with expection to the misc functions). If they operate on doubles then there will be a `d` after the type. For example: `v2_add` for floats and `v2d_add` for doubles. Some functions will also have a `v` or an `a` attached. `m4_translation` and `m4v_translation` are identical functions, only that `m4v` means it takes in a vector. This does get a little ugly. For example `m4dv_translation`, but for now this isn't subject to change.

### Misc Functions

<details>

<summary>Linear Interpolation</summary>

#### Linear Interpolation
```c
float  lerpf(float a, float b, float t);
double lerp(double a, double b, double t)
```
Standard lerp function. Returns a number that is between `a` and `b`. `t` being how far from `a` to "walk" to `b`

</details>

## 2D Vector Functions

### Vector 2D


### Vector Addition
