module LCMPolynomials

using LCMCore
using MultivariatePolynomials
import MultivariatePolynomials: variables, nvariables, polynomial, differentiate
using TypedPolynomials

include("lcmtypes.jl")
include("piecewise.jl")
using .PiecewiseFunctions


function polynomial_t(p::AbstractPolynomial{<:Number}, timestamp=0)
    @assert nvariables(p) == 1
    d = maximum(degree, terms(p))
    coeffs = zeros(d + 1)
    for term in terms(p)
        @assert nvariables(term) == 1  # should be redundant with the nvariables call above, can probably be removed
        coeffs[degree(term) + 1] = coefficient(term) 
    end
    polynomial_t(timestamp, length(coeffs), coeffs)
end

function polynomial_matrix_t(p::AbstractMatrix{<:AbstractPolynomial}, timestamp=0)
    msg = polynomial_matrix_t()
    msg.timestamp = timestamp
    msg.rows = size(p, 1)
    msg.cols = size(p, 2)
    msg.polynomials = [[polynomial_t(p[i, j], timestamp) for j in 1:size(p, 2)] for i in 1:size(p, 1)]
    msg
end

polynomial_matrix_t(p::AbstractVector{<:AbstractPolynomial}, timestamp=0) = polynomial_matrix_t(reshape(p, :, 1), timestamp)


function piecewise_polynomial_t(pf::PiecewiseFunction{<:AbstractPolynomial}, args...)
    piecewise_polynomial_t(fill(pf, 1, 1), args...)
end

function piecewise_polynomial_t(pfs::AbstractVecOrMat{<:PiecewiseFunction{<:AbstractPolynomial}}, timestamp=0)
    pf1 = first(pfs)
    b = breaks(pf1)
    num_pieces = length(b) - 1
    for pf in pfs
        if pf !== pf1
            @assert breaks(pf) == b
            @assert length(pieces(pf)) == num_pieces
        end
    end
    msg = piecewise_polynomial_t(
        timestamp,
        length(b),
        b,
        num_pieces,
        Vector{polynomial_matrix_t}(num_pieces)
    )
    for k in 1:num_pieces
        mat = polynomial_matrix_t()
        mat.rows = size(pfs, 1)
        mat.cols = size(pfs, 2)
        mat.polynomials = [
            [polynomial_t(pieces(pfs[i, j])[k], timestamp) for j in 1:size(pfs, 2)] 
            for i in 1:size(pfs, 1)
        ]
        msg.polynomial_matrices[k] = mat
    end
    msg
end

struct TimePolynomial{T, P<:AbstractPolynomial{T}} <: AbstractPolynomial{T}
    p::P
    function TimePolynomial{T, P}(p::P) where {T, P<:AbstractPolynomial{T}}
        @assert length(variables(p)) == 1
        new{T, P}(p)
    end
end

TimePolynomial(p::P) where {T, P<:AbstractPolynomial{T}} = TimePolynomial{T, P}(p)

(p::TimePolynomial)(x) = p.p(variable(p) => x)
variable(p::TimePolynomial) = first(variables(p))
differentiate(p::TimePolynomial) = TimePolynomial(differentiate(p, first(variables(p))))
polynomial(p::TimePolynomial) = p.p
nvariables(::TimePolynomial) = 1
variables(p::TimePolynomial) = variables(p.p)
Base.show(io::IO, p::TimePolynomial) = print(io, p.p)

function PiecewiseFunctions.PiecewiseFunction(ps::AbstractVector{<:AbstractPolynomial}, breaks::AbstractVector)
    if eltype(ps) <: TimePolynomial
        error("huh?")
    end
    PiecewiseFunction(TimePolynomial.(ps), breaks)
end

PiecewiseFunctions.PiecewiseFunction(ps::AbstractVector{<:TimePolynomial}, breaks::AbstractVector) =
    PiecewiseFunction{eltype(ps), eltype(breaks), typeof(ps), typeof(breaks)}(ps, breaks)

function cubic_spline(Δt, x0, xf, ẋ0, ẋf)
    @polyvar t
    # p(0) = x0
    # p'(0) = x'0
    # p = a + bt + ct^2 + dt^3
    # a = x0
    # b = x'0
    # b + 2ct + 3dt^2 = x'f
    # a + bt + ct^2 + dt^3 = xf
    # bt + 2ct^2 + 3dt^3 = tx'f
    # 3a + 3bt + 3ct^2 + 3dt^3 = 3xf
    # 3a + 2bt + ct^2 = 3xf - tx'f
    a = x0
    b = ẋ0
    c = (3*xf - Δt*ẋf - 3*a - 2*b*Δt) / (Δt^2)
    d = (ẋf - b - 2*c*Δt) / (3*Δt^2)
    TimePolynomial(a + b*t + c*t^2 + d*t^3)
end

function cubic_spline(times::AbstractVector, positions::AbstractVector, velocities::AbstractVector)
    @assert length(times) > 1
    @assert length(times) == length(positions) == length(velocities)
    PiecewiseFunction(
        [cubic_spline(times[i] - times[i-1], positions[i-1], positions[i], velocities[i-1], velocities[i]) for i in 2:length(times)],
        times
    )
end


end # module
