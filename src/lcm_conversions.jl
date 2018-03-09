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
