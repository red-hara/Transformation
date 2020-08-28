# Transformation - basic transformation representation

## Basic elements

### Vec - 3 dimensional vector

```julia-repl
julia> x = Vec(1, 0, 0)
julia> y = Vec(0, 1, 0)
julia> x + y
Vec(1, 1, 0)
julia> cross(x, y)
Vec(0, 0, 1)
```

### Quat - quaternion for rotation representation

Rotate the `(1, 2, 3)` vector using given quaternion:
```julia-repl
julia> q = Quat(2π/3, Vec(1, 1, 1))
julia> v = Vec(1, 2, 3)
julia> round(q * v)
Vec(3.0, 1.0, 2.0)
```

Combine two rotations:
```julia-repl
julia> a = Quat(π/2, Vec(1, 0, 0))
julia> b = Quat(π/2, Vec(0, 1, 0))
julia> a * b
Quat(0.5, 0.5, 0.5, 0.5)
```

### Transf - the combination of consecutive translation and rotation

Calculate the combination of two transformations:
```julia-repl
julia> a = Transf(Vec(0, 0, 2), Quat(-π/2, Vec(1, 0, 0)))
julia> b = Transf(Vec(0, 0, 1))
julia> a + b
Transf(Vec(0.0, 1.0, 2.0), Quat(0.707106781, -0.707106781, 0.0, 0.0))
```

Calculate the vector translated by given transformation:
```julia-repl
julia> t = Transf(Vec(0, 0, 2), Quat(π/2, Vec(1, 0, 0)))
julia> v = Vec(0, 0, 1)
julia> a + b
Vec(0.0, -1.0, 2.0)
```

## Example: modelling the IIWA 14 R820 robot
```julia-repl
julia> using Transformation
julia> function iiwa(q::Array{<:Real,1})::Transf
           y = Vec(0, 1, 0)
           z = Vec(0, 0, 1)
           t0 = Transf(Vec(0, 0, 157.5), Quat(q[1], z))
           t1 = t0 + Transf(Vec(0, 0, 202.5), Quat(q[2], y))
           t2 = t1 + Transf(Vec(0, 0, 204.5), Quat(q[3], z))
           t3 = t2 + Transf(Vec(0, 0, 215.5), Quat(q[4], -y))
           t4 = t3 + Transf(Vec(0, 0, 184.5), Quat(q[5], z))
           t5 = t4 + Transf(Vec(0, 0, 215.5), Quat(q[6], y))
           t6 = t5 + Transf(Vec(0, 0, 81.0), Quat(q[7], z))
           t7 = t6 + Transf(Vec(0, 0, 45.0))
           t7
       end
julia> # Calculate the transformation in `MechanicalZero` position
julia> iiwa([0, 0, 0, 0, 0, 0, 0])
Transf(Vec(0.0, 0.0, 1306.0), Quat(1.0, 0.0, 0.0, 0.0))
julia> # Find out why this model is called R820
julia> iiwa([0, π/2, 0, 0, 0, 0, -π/2, 0])
Transf(Vec(820.0, 0.0, 486.0), Quat(1.0, 0.0, 0.0, 0.0))
```