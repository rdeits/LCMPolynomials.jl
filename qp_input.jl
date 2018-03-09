using Revise

using LCMPolynomials
using AtlasRobot
using HumanoidLCMSim
using LCMCore

include("qptypes.jl")
using .QPTypes

timestamp_now() = Int64(time() * 1e6)

nest(x::AbstractMatrix) = [x[i, :] for i in indices(x, 1)]

function LIPM_zmp_data(x0=zeros(2), y0=zeros(2))
    zmp_data = zmp_data_t()
    zmp_data.A = nest([
        0 0 1 0;
        0 0 0 1;
        0 0 0 0;
        0 0 0 0
    ])
    zmp_data.B = nest([
        0 0;
        0 0;
        1 0;
        0 1;
    ])
    zmp_data.C = nest([
        1 0 0 0;
        0 1 0 0;
    ])
    zmp_data.D = nest([
        -0.101937 0;
        0 -0.101937;
    ])
    zmp_data.x0 = reshape(x0, 2, 1)
    zmp_data.y0 = reshape(y0, 2, 1)
    zmp_data.u0 = zeros(2, 1)
    zmp_data.R = zeros(2, 2)
    zmp_data.Qy = eye(2)
    zmp_data.S = nest(
        0.6386 0      0.2039 0
        0      0.6386 0      0.2039
        0.2039 0      0.0651 0
        0      0.2039 0      0.0651
    )
    zmp_data.s1 = zeros(4, 1)
    zmp_data.s1dot = zeros(4, 1)
    zmp_data.s2 = 0
    zmp_data.s2dot = 0
    zmp_data
end

function foot_support_data(name, surface=@SVector [0, 0, 1, 0], mu=0.7)
    support_data_t(
        timestamp_now(),
        name,
        4,
        nest(@SMatrix [0.1728    0.1728  -0.0876  -0.0876
                       0.0626   -0.0626   0.0626  -0.0626
                       -0.07645 -0.07645 -0.07645 -0.07645]),
        6000,
        5.0,
        ones(SVector{4, Bool})
        true,
        surface,
        mu
    )
end

function body_motion_data(name, splines::AbstractVector{<:PiecewisePolynomial})
    msg = body_motion_data_t()
    msg.timestamp = timestamp_now()
    msg.body_or_frame_name = name
    msg.spline = piecewise_polynomial_t(splines)
    msg.quat_task_to_world = [1, 0, 0, 0]
    msg.translation_task_to_world = [0, 0, 0]
    msg.xyz_kp_multiplier = [1, 1, 1]
    msg.xyz_damping_ratio_multiplier = [1, 1, 1]
    msg.expmap_kp_multiplier = 1
    msg.expmap_damping_ratio_multiplier = 1
    msg.weight_multiplier = [1, 1, 1, 0, 0, 1]
    msg.in_floating_base_nullspace = false
    msg.control_pose_when_in_contact = false
    msg
end

function send_qp_controller_input()
    msg = qp_controller_input_t()
    msg.timestamp = timestamp_now()
    msg.be_silent = false
    msg.zmp_data = LIPM_zmp_data(zeros(2), zeros(2))

    msg.num_support_data = 2
    msg.support_data = [
        foot_support_data("r_foot"),
        foot_support_data("l_foot")
    ]

    msg.num_tracked_bodies = 1
    splines = # TODO: fill me in
    msg.body_motion_data = [body_motion_data("pelvis", splines)]

