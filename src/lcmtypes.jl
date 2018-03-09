mutable struct polynomial_t <: LCMType
    timestamp::Int64
    num_coefficients::Int32
    coefficients::Vector{Float64}
end

@lcmtypesetup(polynomial_t,
    (coefficients, 1) => num_coefficients,
)

mutable struct polynomial_matrix_t <: LCMType
    timestamp::Int64
    rows::Int32
    cols::Int32
    polynomials::Vector{Vector{polynomial_t}}
end

@lcmtypesetup(polynomial_matrix_t,
    (polynomials, 1) => rows,
    (polynomials, 2) => cols
)

mutable struct piecewise_polynomial_t  <: LCMType
    timestamp::Int64
    num_breaks::Int32
    breaks::Vector{Float64}
    num_segments::Int32
    polynomial_matrices::Vector{polynomial_matrix_t}
end

@lcmtypesetup(piecewise_polynomial_t,
    (breaks, 1) => num_breaks,
    (polynomial_matrices, 1) => num_segments,
)
