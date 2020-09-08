module Transformation

export Vec, Quat, Transf, dot, cross, magnitude, norm, ×, slerp, unit

"""
    Vec - three dimensional vector.

# Examples
```julia-repl
julia> Vec(1, 2, 3)
Vec(1, 2, 3)
```
"""
struct Vec
    x::Real
    y::Real
    z::Real
    function Vec(x::Real=0, y::Real=0, z::Real=0)::Vec
        new(x, y, z)
    end
end

function Base.:+(a::Vec, b::Vec)::Vec
    Vec(
        a.x + b.x,
        a.y + b.y,
        a.z + b.z
    )
end

function Base.:-(a::Vec, b::Vec)::Vec
    Vec(
        a.x - b.x,
        a.y - b.y,
        a.z - b.z
    )
end

function Base.:-(a::Vec)::Vec
    Vec(
        -a.x,
        -a.y,
        -a.z
    )
end

"""
    dot(a::Vec, b::Vec)::Vec

Compute the dot product of two vectors.
"""
function dot(a::Vec, b::Vec)
    a.x * b.x + a.y * b.y + a.z * b.z
end

"""
    cross(a::Vec, b::Vec)::Vec

Compute the cross product of two vectors.
"""
function cross(a::Vec, b::Vec)::Vec
    Vec(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    )
end

"""
    ×(a::Vec, b::Vec)::Vec

Compute the cross product of two vectors.
"""
function ×(a::Vec, b::Vec)::Vec
    cross(a, b)
end

function Base.:*(a::Real, b::Vec)::Vec
    Vec(
        a * b.x,
        a * b.y,
        a * b.z
    )
end

function Base.:*(a::Vec, b::Real)::Vec
    Vec(
        a.x * b,
        a.y * b,
        a.z * b
    )
end

function Base.:/(a::Vec, b::Real)::Vec
    Vec(
        a.x / b,
        a.y / b,
        a.z / b
    )
end

function Base.isequal(a::Vec, b::Vec)::Bool
    a.x == b.x && a.y == b.y && a.z == b.z
end

"""
    norm(a::Vec)::Real

Compute the Euclidean norm of the given vector.
"""
function norm(a::Vec)::Real
    √(a.x^2 + a.y^2 + a.z^2)
end

"""
    unit(a::Vec)::Vec

Compute the unit vector colinear to the given one.
"""
function unit(a::Vec)::Vec
    a / norm(a)
end

"""
    round(a::Vec)::Vec

Compute the vector with it's elements rounder to nearest value.
"""
function Base.round(a::Vec)::Vec
    Vec(
        round(a.x),
        round(a.y),
        round(a.z)
    )
end

"""
    Quat - hypercomplex number.

Unit quaternions are used for rotation representation.

# Examples
```julia-repl
julia> # 120° counterclockwise rotation around the (1, 1, 1) axis
julia> Quat(2π/3, Vec(1, 1, 1))
Quat(0.5, 0.5, 0.5, 0.5)

julia> # 45° clockwise rotation around the (1, 0, 0) axis
julia> Quat(-π/4, Vec(1, 0, 0))
Quat(0.9238795325112867, -0.3826834323650898, -0.0, -0.0)
```
"""
struct Quat
    w::Real
    x::Real
    y::Real
    z::Real
    function Quat(w::Real=1, x::Real=0, y::Real=0, z::Real=0)::Quat
        new(w, x, y, z)
    end
    
    function Quat(angle::Real, axis::Vec)::Quat
        w = cos(angle / 2)
        xyz = unit(axis) * sin(angle / 2)
        Quat(w, xyz.x, xyz.y, xyz.z)
    end
end

function Base.:+(a::Quat, b::Quat)::Quat
    Quat(
        a.w + b.w,
        a.x + b.x,
        a.y + b.y,
        a.z + b.z
    )
end

function Base.:-(a::Quat, b::Quat)::Quat
    Quat(
        a.w - b.w,
        a.x - b.x,
        a.y - b.y,
        a.z - b.z
    )
end

function Base.:*(a::Quat, b::Quat)::Quat
    Quat(
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    )
end

function Base.:*(q::Quat, v::Vec)::Vec
    r = q * Quat(0, v.x, v.y, v.z) * conj(q)
    Vec(r.x, r.y, r.z)
end

function Base.:*(a::Quat, b::Real)::Quat
    Quat(
        a.w * b,
        a.x * b,
        a.y * b,
        a.z * b
    )
end

function Base.:*(a::Real, b::Quat)::Quat
    Quat(
        a * b.w,
        a * b.x,
        a * b.y,
        a * b.z
    )
end

function dot(a::Quat, b::Quat)::Real
    a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z
end

function Base.:/(a::Quat, b::Real)::Quat
    Quat(
        a.w / b,
        a.x / b,
        a.y / b,
        a.z / b
    )
end

"""
    conj(a::Quat)::Quat

Compute the quaternion conjugate.
If the quaternion is unit, it's conjugate represents the reverse rotation.

# Examples
```julia-repl
julia> q = Quat(π, Vec(1, 1, 1))
julia> q * conj(q)
Quat(1.0, 0.0, 0.0, 0.0)
```
"""
function Base.conj(a::Quat)::Quat
    Quat(
        a.w,
        -a.x,
        -a.y,
        -a.z
    )
end

"""
    norm(a::Quat)::Real

Compute the quaternion norm.
"""
function norm(a::Quat)::Real
    √(a.w^2 + a.x^2 + a.y^2 + a.z^2)
end

"""
    unit(a::Quat)::Quat

Compute the unit quaternion colinear to the given one.
"""
function unit(a::Quat)::Quat
    a / norm(a)
end

function slerp(a::Quat, b::Quat, t)::Quat
    d = dot(a, b)
    if d < 0
        b = -b
        d = -d
    end
    if d > 0.9995
        result = a + (a - b) * t
        return norm(result)
    end
    theta0 = acos(d)
    theta = theta0 * t
    sinTheta = sin(theta)
    sinTheta0 = sin(theta0)

    s1 = sinTheta / sinTheta0
    s0 = cos(theta) - d * s1
    return a * s0 + b * s1
end

"""
    Transf - spatial transformation representation.

The transformation consists of consecutive translation on vector `v` and rotation on `q` quaternion.
"""
struct Transf
    v::Vec
    q::Quat
    function Transf(v::Vec=Vec(), q::Quat=Quat())::Transf
        new(v, q)
    end
end

function Base.:+(a::Transf, b::Transf)::Transf
    Transf(
        a.v + a.q * b.v,
        a.q * b.q
    )
end

function Base.:+(a::Transf, b::Vec)::Vec
    a.v + a.q * b
end

"""
    conj(a::Transf)::Transf

Compute such transformation, that `a + conj(a)` equals to zero transformation.

# Examples
```julia-repl
julia> t = Transf(Vec(1, 2, 3), Quat(π/3, Vec(1, 1, 0)))
julia> t + conj(t)
Transf(Vec(0.0, 0.0, 0.0), Quat(1.0, 0.0, 0.0, 0.0))
```
"""
function Base.conj(a::Transf)::Transf
    q = conj(a.q)
    Transf(
        q * (-a.v),
        q
    )
end

end
