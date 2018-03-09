module QPTypes

using LCMCore

export zmp_data_t,
       support_data_t,
       body_motion_data_t,
       body_wrench_t,
       whole_body_data_t,
       joint_pd_override_t,
       qp_controller_input_t

mutable struct zmp_data_t <: LCMType
    timestamp::Int64
    A::SVector{4, SVector{4, Float64}}
    B::SVector{4, SVector{2, Float64}}
    C::SVector{2, SVector{4, Float64}}
    D::SVector{2, SVector{2, Float64}}
    x0::SVector{4, SVector{1, Float64}}
    y0::SVector{2, SVector{1, Float64}}
    u0::SVector{2, SVector{1, Float64}}
    R::SVector{2, SVector{2, Float64}}
    Qy::SVector{2, SVector{2, Float64}}
    S::SVector{4, SVector{4, Float64}}
    s1::SVector{4, SVector{1, Float64}}
    s1dot::SVector{4, SVector{1, Float64}}
    s2::Float64
    s2dot::Float64

    com::SVector{4, SVector{1, Float64}}
end

@lcmtypesetup(zmp_data_t)


mutable struct support_data_t <: LCMType
    timestamp::Int64
    body_name::String
    num_contact_pts::Int32
    contact_pts::SVector{3, Vector{Float64}}
    total_normal_force_upper_bound::Float32
    total_normal_force_lower_bound::Float32
    support_logic_map::SVector{4, Bool}
    use_support_surface::Bool
    support_surface::SVector{4, Float32}
    mu::Float64
end

@lcmtypesetup(support_data_t,
    (contact_pts, 2) => num_contact_pts
)

mutable struct body_motion_data_t <: LCMType
    timestamp::Int64
    body_or_frame_name::String
    spline::piecewise_polynomial_t
    quat_task_to_world::SVector{4, Float64}
    translation_task_to_world::Svector{3, Float64}
    xyz_kp_multiplier::SVector{3, Float64}
    xyz_damping_ratio_multiplier::SVector{3, Float64}
    expmap_kp_multiplier::Float64
    expmap_damping_ratio_multiplier::Float64
    weight_multiplier::SVector{6, Float64}
    in_floating_base_nullspace::Bool
    control_pose_when_in_contact::Bool
end

@lcmtypesetup(body_motion_data_t)

mutable struct body_wrench_t <: LCMType
    timestamp::Int64
    body_name::String
    wrench::SVector{6, Float64}
end

@lcmtypesetup(body_wrench_t)

mutable struct whole_body_data_t <: LCMType
    timestamp::Int64
    num_positions::Int32
    q_des::Vector{Float32}
    splint::piecewise_polynomial_t
    num_constrained_dofs::Int32
    constrained_dofs::Vector{Int32}
end

@lcmtypesetup(whole_body_data_t,
    (q_des, 1) => num_positions,
    (constrained_dofs, 1) => num_constrained_dofs
)

mutable struct joint_pd_override_t
    timestamp::Int64
    position_ind::Int32
    qi_des::Float64
    qdi_des::Float64
    kp::Float64
    kd::Float64
    weight::Float64
end

@lcmtypesetup(joint_pd_override_t)

mutable struct qp_controller_input_t
    be_silent::Bool
    timestamp::Int64
    zmp_data::zmp_data_t

    num_support_data::Int32
    support_data::Vector{support_data_t}

    num_tracked_bodies::Int32
    body_motion_data::Vector{body_motion_data_t}

    num_external_wrenches::Int32
    body_wrench_data::Vector{body_wrench_data_t}

    whole_body_data::whole_body_data_t

    num_joint_pd_overrides::Int32
    joint_pd_overrides::Vector{joint_pd_override_t}

    param_set_name::String

    torque_alpha_filter::Float64
end

@lcmtypesetup(qp_controller_input_t,
    (support_data, 1) => num_support_data,
    (body_motion_data, 1) => num_tracked_bodies,
    (body_wrench_data, 1) => num_external_wrenches,
    (joint_pd_overrides, 1) => num_joint_pd_overrides,
)

end
