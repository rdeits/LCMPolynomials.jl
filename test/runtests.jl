using LCMPolynomials
using TypedPolynomials
using Base.Test

@testset "LCMPolynomials" begin
    @testset "cubic spline segments" begin
        srand(1)
        for i in 1:1000
            x0 = randn()
            xd0 = randn()
            xf = randn()
            xdf = randn()
            dt = randn()
            p = cubic_spline(dt, x0, xf, xd0, xdf)
            @test isapprox(p(0), x0)
            @test isapprox(p(dt), xf)
            pd = differentiate(p)
            @test isapprox(pd(0), xd0)
            @test isapprox(pd(dt), xdf)
        end
    end

    @testset "cubic splines" begin
        srand(10)
        for i in 1:1000
            N = rand(2:10)
            dts = rand(N - 1) .+ 0.1
            ts = vcat(0, cumsum(dts))
            xs = randn(N)
            xds = randn(N)
            pp = cubic_spline(ts, xs, xds)
            ppd = differentiate.(pp)
            for i in 1:N-1
                @test isapprox(pp(ts[i]), xs[i])
                @test isapprox(ppd(ts[i]), xds[i])
            end
            @test isapprox(pp(ts[end] - 1e-12), xs[end], rtol=1e-6)
            @test isapprox(ppd(ts[end] - 1e-12), xds[end], rtol=1e-6)
            @test isapprox(LCMPolynomials.from_below(pp, ts[end]), xs[end])
            @test isapprox(LCMPolynomials.from_below(ppd, ts[end]), xds[end])
        end
    end

    @testset "LCM construction" begin
        N = rand(2:10)
        dts = rand(N - 1) .+ 0.1
        ts = vcat(0, cumsum(dts))
        xs = randn(N)
        xds = randn(N)
        pp = cubic_spline(ts, xs, xds)
        msg = piecewise_polynomial_t(pp)
        msg2 = piecewise_polynomial_t([pp, pp])
    end
end
