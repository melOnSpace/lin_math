
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

All functions are prefixed with the type they operate on (with expection to the misc functions). If they operate on doubles then there will be a `d` after the type. For example: `v2_add` for floats and `v2d_add` for doubles. Some functions will also have a `v` attached. `m4_translation` and `m4v_translation` are identical functions, only that `m4v` means it takes in a vector. This does get a little ugly. For example `m4dv_translation`, but for now this isn't subject to change.

## Misc Functions

#### ***Fast Inverse Square Root***

```c
float  fast_rsqrtf(float number);
double fast_rsqrt(double number);
```
<details>
<summary>Description</summary>

[Wikipedia Article on the function](https://en.wikipedia.org/wiki/Fast_inverse_square_root)

</details>

#### ***Linear Interpolation***

```c
float  lerpf(float a, float b, float t);
double lerp(double a, double b, double t);
```
<details>
<summary>Description</summary>

Standard lerp function. Returns a number that is between `a` and `b`. `t` being how far from `a` to "walk" to `b`

[Wikipedia Article about linear interpolation](https://en.wikipedia.org/wiki/Linear_interpolation)

</details>

#### ***Casting from Type A to Type B***

#### Cast to 2D Vector
```c
v2_t v3_to_v2(v3_t v);
v2_t v4_to_v2(v4_t v);
v2_t qt_to_v2(qt_t q);
```

#### Cast to 3D Vector
```c
v3_t v2_to_v3(v2_t v);
v3_t v4_to_v3(v4_t v);
v3_t qt_to_v3(qt_t q);
```

#### Cast to 4D Vector
```c
v4_t v2_to_v4(v2_t v);
v4_t v3_to_v4(v3_t v);
v4_t qt_to_v4(qt_t q);
```

#### Cast to Quaternion
```c
qt_t v2_to_qt(v2_t v);
qt_t v3_to_qt(v3_t v);
qt_t v4_to_qt(v4_t v);
```

#### ***Printing Types with `printf`***
```c
#define v2_print(v) (printf("[%f, %f]", (v).x, (v).y))
#define v3_print(v) (printf("[%f, %f, %f]", (v).x, (v).y, (v).z))
#define v4_print(v) (printf("[%f, %f, %f, %f]", (v).x, (v).y, (v).z, (v).w))

#define v2_println(v) (printf("[%f, %f]\n", (v).x, (v).y))
#define v3_println(v) (printf("[%f, %f, %f]\n", (v).x, (v).y, (v).z))
#define v4_println(v) (printf("[%f, %f, %f, %f]\n", (v).x, (v).y, (v).z, (v).w))

#define qt_print(q)   (printf("[%f, %fi, %fj, %fk]",   (q).r, (q).i, (q).j, (q).k))
#define qt_println(q) (printf("[%f, %fi, %fj, %fk\n]", (q).r, (q).i, (q).j, (q).k))

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

```
<details>
<summary>Description</summary>

Macros! Because these are macros they can take in either `float` or `double` types. One day I will write a wrapper for `printf` for better functionality, but until then this will do.

</details>

## 32-bit Float Vector Functions

#### ***Vector Builder***

```c
v2_t vec2(float x, float y);
v3_t vec3(float x, float y, float z);
v4_t vec4(float x, float y, float z, float w);
```

#### ***Vector Initialization***

```c
v2_t vec2Init(float i);
v3_t vec3Init(float i);
v4_t vec4Init(float i);
```
<details>
<summary>Description</summary>
Returns a vector with all fields initialized to `i`

</details>

#### ***Basic 2D Vector Arithmetic***

#### Pair-Wise Operators
```c
v2_t v2_add(v2_t a, v2_t b); /* Addition */
v3_t v3_add(v3_t a, v3_t b);
v4_t v4_add(v4_t a, v4_t b);

v2_t v2_sub(v2_t a, v2_t b); /* Subtraction */
v3_t v3_sub(v3_t a, v3_t b);
v4_t v4_sub(v4_t a, v4_t b);

v2_t v2_mul(v2_t a, v2_t b); /* Multiplication */
v3_t v3_mul(v3_t a, v3_t b);
v4_t v4_mul(v4_t a, v4_t b);

v2_t v2_div(v2_t a, v2_t b); /* Division */
v3_t v3_div(v3_t a, v3_t b);
v4_t v4_div(v4_t a, v4_t b);
```

#### Scalar Operators
```c
v2_t v2_adds(v2_t v, float s); /* Addition */
v3_t v3_adds(v3_t v, float s);
v4_t v4_adds(v4_t v, float s);

v2_t v2_subs(v2_t v, float s); /* Subtraction */
v3_t v3_adds(v3_t v, float s);
v4_t v4_adds(v4_t v, float s);

v2_t v2_muls(v2_t v, float s); /* Multiplication */
v3_t v3_adds(v3_t v, float s);
v4_t v4_adds(v4_t v, float s);

v2_t v2_divs(v2_t v, float s); /* Division */
v3_t v3_adds(v3_t v, float s);
v4_t v4_adds(v4_t v, float s);
```
#### Variatic Functions
```c
v2_t v2_sum(int num, ...); /* Addition */
v3_t v3_sum(int num, ...);
v4_t v4_sum(int num, ...);

v2_t v2_difference(int num, ...); /* Subtraction */
v3_t v3_difference(int num, ...);
v4_t v4_difference(int num, ...);

v2_t v2_product(int num, ...); /* Multiplication */
v3_t v3_product(int num, ...);
v4_t v4_product(int num, ...);

v2_t v2_quotient(int num, ...); /* Division */
v3_t v3_quotient(int num, ...);
v4_t v4_quotient(int num, ...);
```
- `num` must be the amount of vectors passed
#### Array Functions
```c
v2_t v2_sum_array(size_t len, v2_t* array); /* Addition */
v3_t v3_sum_array(size_t len, v3_t* array);
v4_t v4_sum_array(size_t len, v4_t* array);

v2_t v2_difference_array(size_t len, v2_t* array); /* Subtraction */
v3_t v3_difference_array(size_t len, v3_t* array);
v4_t v4_difference_array(size_t len, v4_t* array);

v2_t v2_product_array(size_t len, v2_t* array); /* Multiplication */
v3_t v3_product_array(size_t len, v3_t* array);
v4_t v4_product_array(size_t len, v4_t* array);

v2_t v2_quotient_array(size_t len, v2_t* array); /* Division */
v3_t v3_quotient_array(size_t len, v3_t* array);
v4_t v4_quotient_array(size_t len, v4_t* array);
```
- These functions are implemented via loops that iterate over `array`. Be careful to pass the correct `len`, *especially* if `array` is allocated on the stack!

#### ***Other 2D Vector Arithmetic***

#### Projection
```c
v2_t v2_project(v2_t a, v2_t b);
v3_t v3_project(v3_t a, v3_t b);
v4_t v4_project(v4_t a, v4_t b);
```
<details>
<summary>Description</summary>

Projects vector `a` onto `b`. A common explanation is to think of vector `a` casting a shadow onto `b`.

[Better Explanation that what I just gave (i.e., Wikipedia)](https://en.wikipedia.org/wiki/Vector_projection)

</details>

#### Normalization
```c
v2_t v2_norm(v2_t v); v2_t v2_fastnorm(v2_t v);
v3_t v3_norm(v3_t v); v3_t v3_fastnorm(v3_t v);
v4_t v4_norm(v4_t v); v4_t v4_fastnorm(v4_t v);

v4_t v4_normAxis(v4_t v); v4_t v4_fastnormAxis(v4_t v);
```
<details>
<summary>Description</summary>

A normalize function returns a vector pointing in the same direction but with a length always of 1
The difference between `norm` and `fastnorm` is the square-root operator they both use. The norm fucntion uses `math.h sqrtf()`; fast_norm uses the `fast_rsqrtf()` function.

[Fast Inverse Square Root](https://en.wikipedia.org/wiki/Fast_inverse_square_root)

The `v4_nromAxis` functions exist for Axis-Angle rotations, which require a normalized to function. `v4_normAxis` will treat `v` as an Axis-Angle and only normalize the `v.axis` part of the vector, leaving `v.angle` alone.

</details>

#### Linear Interpolation of Vectors
```c
v2_t v2_lerp(v2_t a, v2_t b, float t);
v3_t v3_lerp(v3_t a, v3_t b, float t);
v4_t v4_lerp(v4_t a, v4_t b, float t);
```
<details>
<summary>Description</summary>

Standard lerp function. Returns a number that is between `a` and `b`. `t` being how far from `a` to "walk" to `b`

[Wikipedia Article about linear interpolation](https://en.wikipedia.org/wiki/Linear_interpolation)

</details>

#### Cross Product
```c
v3_t v3_cross(v3_t a, v3_t b);
```
<details>
<summary>Description</summary>

Note that there is only a 3D vector cross product. That is because the cross product does not exist for 2D or 4D vectors. If the cross product of two axis angles are wanted, one can do this: `v3_t new_angle = v3_cross(axis_a.axis, axis_b.axis);`

[Wikipedia Article about cross product](https://en.wikipedia.org/wiki/Cross_product)

</details>

#### Dot Product
```c
float v2_dot(v2_t a, v2_t b);
float v3_dot(v3_t a, v3_t b);
float v4_dot(v4_t a, v4_t b);
```
<details>
<summary>Description</summary>

[Wikipedia Article about Dot Product](https://en.wikipedia.org/wiki/Dot_product)

</details>

#### Vector Magnitude (i.e., Length of Vectors)
```c
float v2_mag(v2_t v);           float v2_fastmag(v2_t v);
float v3_mag(v3_t v);           float v3_fastmag(v3_t v);
float v4_mag(v4_t v);           float v4_fastmag(v4_t v);

float v2_dist(v2_t a, v2_t b);  float v2_fastdist(v2_t a, v2_t b);
float v3_dist(v3_t a, v3_t b);  float v3_fastdist(v3_t a, v3_t b);
float v4_dist(v4_t a, v4_t b);  float v4_fastdist(v4_t a, v4_t b);
```
<details>
<summary>Description</summary>

There are two types of functions for getting the magnitude of a vector; both of which have fast variants. 

|**Function**|**Description**             |**Fast Variant**                      |
|:---------- |:-------------------------- |:------------------------------------ |
|`mag`       | The length of vector `v`   | The length<sup>2</sup> of vector `v` |
|`dist`      | Short-hand for `mag(a - b)`| Short-hand for `fastmag(a - b)`      |

</details>

#### Angle Between Vectors
```c
v2_t v2_angle(v2_t a, v2_t b);
v3_t v3_angle(v3_t a, v3_t b);
v4_t v4_angle(v4_t a, v4_t b);
```
<details>
<summary>Description</summary>

Returns the angle between vectors `a` and `b`. For example: the angle between `{ 1.0, 0.0 }` and `{ 0.0, 1.0 }` is $π / 2$

Like all functions in this library, angles are expressed in radians

</details>

#### Check if the Magnitude of a Vector is Zero
```c
int v2_isZero(v2_t v);
int v3_isZero(v3_t v);
int v4_isZero(v4_t v);
```
<details>
<summary>Description</summary>

Calculated the magnitude of `v` and returns `0` if it is zero.
The function does not actually compare `mag(v)` with zero. `isZero` does the following `return fastmag(v) < LIN_EPSILON_F`

Floating point numbers are weird, so it is often easier to compare to a number called epsilon instead of zero. Especially because floating point numbers can be negative zero.

</details>

## 32-bit Quaternion Functions

#### ***Quaternion Builder***
```c
qt_t quat(float r, float i, float j, float k);
qt_t quatInit(float i);
```
<details>
<summary>Description</summary>

`quat` returns a quaternion from `r`, `i`, `j`, `k`.
`quatInit` returns a quaternion with all fields set to `i`.

</details>

#### ***Quaternion Arithmetic***

#### Pair-Wise Operators
```c
qt_t qt_add(qt_t a, qt_t b);
qt_t qt_sub(qt_t a, qt_t b);
qt_t qt_mul(qt_t a, qt_t b);
```
- Note the lack of Quaternion division. This is deliberate. For quaternion division: $Q = p/q$ is the same as $Q = p^{-1} * q$

#### Scalar Operators
```c
qt_t qt_adds(qt_t q, float s);
qt_t qt_subs(qt_t q, float s);
qt_t qt_muls(qt_t q, float s);
qt_t qt_divs(qt_t q, float s);
```

#### Variatic and Array Functions
```c
qt_t qt_sum(int num, ...); /* Addition */
qt_t qt_difference(int num, ...); /* Subtraction */
qt_t qt_product(int num, ...); /* Muliplication */

qt_t qt_sum_array(size_t len, qt_t* array); /* Addition */
qt_t qt_difference_array(size_t len, qt_t* array); /* Subtraction */
qt_t qt_product_array(size_t len, qt_t* array); /* Muliplication */
```
- Variatic functions take `num`, which must be the amount of quaternions passed
- Array functions are implemented via loops that iterate over `array`. Be careful to pass the correct `len`, *especially* if `array` is allocated on the stack!

#### ***Other Quaternion Arithmetic***

#### Square Root
```c
qt_t qt_sqrt(qt_t q);
```
<details>
<summary>Description</summary>

Returns the square root of `q`. Note that the return is a quaternion. [For more info about taking square roots of complex numbers.](https://en.wikipedia.org/wiki/Quaternion#Square_roots_of_arbitrary_quaternions)

</details>

#### Exponential
```c
qt_t qt_exp(qt_t q);
```
<details>
<summary>Description</summary>

Returns $e^q$

[Here is a Wikipedia Article about the topic.](https://en.wikipedia.org/wiki/Quaternion#Exponential,_logarithm,_and_power_functions)

</details>

#### Natural Logarithm
```c
qt_t qt_ln(qt_t q);
```
<details>
<summary>Description</summary>

Returns $ln(q)$. If other log bases are wanted then one could do this: 

$$p = \log_b(q)$$
$$p = \frac{ln(q)}{ln(b)}$$
$$p = (ln(b))^{-1} * ln(q)$$
Where q, b, and p are all quaternions

```c
// Example function. This in not in the library
qt_t qt_log(qt_t q, qt_t base) {
    qt_t ilog_base = qt_inverse(qt_ln(base));
    return qt_mul(ilog_base, qt_ln(q));
}
```

[Yet another Wikipedia Article.](https://en.wikipedia.org/wiki/Quaternion#Exponential,_logarithm,_and_power_functions)

</details>

#### Quaternion Raised to a Real Number
```c
qt_t qt_pows(qt_t q, float s);
```
<details>
<summary>Description</summary>

[My source for the function](https://math.stackexchange.com/questions/939229/unit-quaternion-to-a-scalar-power)

</details>

#### ****More Quaternion Arithmetic****

#### Quaternion Conjugate
```c
qt_t qt_conjugate(qt_t q);
```
<details>
<summary>Description</summary>

Does the following: 
$$q = a + bi + cj + dk$$
$$conjugate(q) = a - bi - cj - dk$$

</details>
