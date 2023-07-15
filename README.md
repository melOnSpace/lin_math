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

# API Reference

## Types

| **Type** | **Double Variant** | **Description** |
| :------- | :-------------- | :----------------- |
| v2_t     | v2_d            | 2D Vector          |
| v3_t     | v3_d            | 3D Vector          |
| v4_t     | v4_d            | 4D Vector          |
| qt_t     | qt_d            | Quaternion         |
| m4x4_t   | m4x4_d          | 4 by 4 Matrix      |

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

#### Get item

```c
  GET /api/items/${id}
```

| Parameter | Type     | Description                       |
| :-------- | :------- | :-------------------------------- |
| `id`      | `string` | **Required**. Id of item to fetch |

#### add(num1, num2)

Takes two numbers and returns the sum.

