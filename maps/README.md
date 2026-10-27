# Arena maps

The two map files describe the competition arenas. They use the course
framework format:

```
sizeX sizeY sizeCell
<c> <c> <c> ...   (2 * sizeY + 1 rows)
```

* `sizeX`, `sizeY` — number of cells.
* `sizeCell` — cell side in **millimetres** (400 mm = 40 cm here).
* The connection matrix has `2 * sizeX + 1` columns and `2 * sizeY + 1` rows.
  Odd indices are cell centres; even indices are wall segments. `1` is open,
  `0` is blocked.
* File rows are written from the top of the arena down. The loader stores each
  row as a column of the connection matrix, which is why the axes look
  transposed when reading the file directly.

| File | Floor | Start cell | Goal cell | Assigned marker |
|---|---|---|---|---|
| `arena_a.txt` | white | `(1, 2)` | `(3, 3)` | R2-D2 |
| `arena_b.txt` | black | `(5, 2)` | `(3, 3)` | BB-8 |

Cell `(x, y)` has its centre at `((x + 0.5) * 40, (y + 0.5) * 40)` cm, with the
origin at the bottom-left corner of the arena. For example, `(1, 2)` is
`(60, 100)` cm, which is exactly the robot's initial pose in
`robot/config.py`.

The planner is four-connected (north/east/south/west), matching the movement
primitives used by the robot.
