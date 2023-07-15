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

