module LCMPolynomials

using LCMCore
using MultivariatePolynomials
import MultivariatePolynomials: variables, nvariables, polynomial, differentiate
using TypedPolynomials

export polynomial_t,
       polynomial_matrix_t,
       piecewise_polynomial_t,
       PiecewiseFunction,
       cubic_spline

include("lcmtypes.jl")
include("piecewise.jl")
using .PiecewiseFunctions
include("lcm_conversions.jl")
include("splines.jl")

end # module
