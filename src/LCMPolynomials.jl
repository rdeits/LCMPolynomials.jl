module LCMPolynomials

using LCMCore
using DrakeLCMTypes: polynomial_t, piecewise_polynomial_t, polynomial_matrix_t
using MultivariatePolynomials
import MultivariatePolynomials: variables, nvariables, polynomial, differentiate
using TypedPolynomials

export polynomial_t,
       polynomial_matrix_t,
       piecewise_polynomial_t,
       PiecewiseFunction,
       PiecewisePolynomial,
       cubic_spline

include("piecewise.jl")
using .PiecewiseFunctions

const PiecewisePolynomial = PiecewiseFunction{<:AbstractPolynomial}

include("lcm_conversions.jl")
include("splines.jl")

end # module
