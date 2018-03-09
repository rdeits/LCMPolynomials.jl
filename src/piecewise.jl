module PiecewiseFunctions
import MultivariatePolynomials: AbstractVariable, variable
export PiecewiseFunction, from_above, from_below, pieces, breaks

struct PiecewiseFunction{F, B<:Real, FV<:AbstractVector{F}, BV<:AbstractVector{B}} <: Function
    pieces::FV
    breaks::BV

    function PiecewiseFunction{F, B, FV, BV}(pieces::FV, breaks::BV) where {F, B, FV<:AbstractVector{F}, BV<:AbstractVector{B}}
        @assert issorted(breaks)
        @assert length(breaks) == length(pieces) + 1
        new{F, B, FV, BV}(pieces, breaks)
    end
end

PiecewiseFunction(pieces::FV, breaks::BV) where {F, B, FV<:AbstractVector{F}, BV<:AbstractVector{B}} = 
    PiecewiseFunction{F, B, FV, BV}(pieces, breaks)

breaks(p::PiecewiseFunction) = p.breaks
pieces(p::PiecewiseFunction) = p.pieces
(pf::PiecewiseFunction)(t) = from_above(pf, t)

function from_above(pf::PiecewiseFunction, t)
    i = searchsortedlast(pf.breaks, t)
    if i <= 0 || i >= length(pf.breaks)
        error("Input value $t is out of the allowable range [$(pf.breaks[1]), $(pf.breaks[end]))")
    end
    pf.pieces[i](t - pf.breaks[i])
end

function from_below(pf::PiecewiseFunction, t)
    i = searchsortedfirst(pf.breaks, t)
    if i <= 1 || i >= length(pf.breaks) + 1
        error("Input value $t is out of the allowable range ($(pf.breaks[1]), $(pf.breaks[end])]")
    end
    pf.pieces[i - 1](t - pf.breaks[i - 1])
end

Base.broadcast(f, pf::PiecewiseFunction) = PiecewiseFunction(f.(pieces(pf)), breaks(pf))
end