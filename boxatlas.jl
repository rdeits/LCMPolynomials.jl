using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics
using RigidBodyDynamics: Bounds
using LCMCore
using BotCoreLCMTypes
using StaticArrays

function planar_base()
    world = RigidBody{Float64}("world")
    mechanism = Mechanism(world; gravity=SVector(0, 0, -9.81))

    frame = CartesianFrame3D("dummy")
    inertia = SpatialInertia(frame, 0 * eye(3), zeros(3), 0.0)
    dummy = RigidBody(inertia)
    base_x = Joint("base_x", Prismatic([1., 0, 0]))
    position_bounds(base_x) .= Bounds(-10., 10)
    velocity_bounds(base_x) .= Bounds(-1000., 1000)
    effort_bounds(base_x) .= Bounds(0., 0)
    attach!(mechanism, world, dummy, base_x)

    frame = CartesianFrame3D("base")
    inertia = SpatialInertia(frame, 0 * eye(3), zeros(3), 0.0)
    base = RigidBody(inertia)
    base_z = Joint("base_z", Prismatic([0., 0, 1]))
    position_bounds(base_z) .= Bounds(-10., 10)
    velocity_bounds(base_z) .= Bounds(-1000., 1000)
    effort_bounds(base_z) .= Bounds(0., 0)
    attach!(mechanism, dummy, base, base_z)
    mechanism, base
end

function planar_revolute_base()
    mechanism, base = planar_base()
    frame = CartesianFrame3D("base_revolute")
    inertia = SpatialInertia(frame, 0 * eye(3), zeros(3), 0.0)
    body = RigidBody(inertia)
    joint = Joint("base_rotation", Revolute([0., 1., 0]))
    position_bounds(joint) .= Bounds(-2π, 2π)
    velocity_bounds(joint) .= Bounds(-1000., 1000)
    effort_bounds(joint) .= Bounds(0., 0)
    attach!(mechanism, base, body, joint)
    mechanism, body
end

urdf = joinpath(@__DIR__, "box_atlas.urdf")
urdf_mech = parse_urdf(Float64, urdf)
robot, base_body = planar_revolute_base()
attach!(robot, base_body, urdf_mech)
state = MechanismState(robot)

vis = Visualizer("tcp://127.0.0.1:6000")
mvis = MechanismVisualizer(robot, URDFVisuals(urdf), vis)
open(mvis.visualizer)
wait(mvis.visualizer)

# while true
#     set_configuration!(mvis, randn(num_positions(state)))
#     sleep(0.001)
# end

configurations = Channel{Any}(32)

lcm = LCM()
function on_robot_state(channel::String, msg::robot_state_t)
    set_configuration!(state, findjoint(robot, "base_x"), [-msg.pose.translation.y])
    set_configuration!(state, findjoint(robot, "base_z"), [msg.pose.translation.z])

    world = default_frame(root_body(robot))
    com_position = FreeVector3D(world,
        0,
        msg.pose.translation.y,
        msg.pose.translation.z
    )
    hip_offset = FreeVector3D(world, # TODO: not actually in world
        0, 0.2, -0.25
    )
    foot_position = FreeVector3D(world,
        0,
        msg.joint_position[8],
        msg.joint_position[9]
    )
    l_hip_to_lf = foot_position - (com_position + hip_offset)
    set_configuration!(state, findjoint(robot, "core_to_lf_extension"), [norm(l_hip_to_lf)])
    θ = π - atan2(l_hip_to_lf.v[2], l_hip_to_lf.v[3])
    set_configuration!(state, findjoint(robot, "core_to_lf_rotation"), [θ])

    rf_position = FreeVector3D(world, 0, msg.joint_position[11], msg.joint_position[12])
    r_hip_position = com_position + FreeVector3D(world, 0, -0.2, -0.25) # TODO: not in world
    r_hip_to_rf = r_hip_position - rf_position
    set_configuration!(state, findjoint(robot, "core_to_rf_extension"), [norm(r_hip_to_rf)])
    θ = atan2(r_hip_to_rf.v[2], r_hip_to_rf.v[3])
    set_configuration!(state, findjoint(robot, "core_to_rf_rotation"), [θ])

    # set_configuration!(mvis, configuration(state))
    put!(configurations, copy(configuration(state)))
end

@sync begin
    @async while true
        q = take!(configurations)
        set_configuration!(mvis, q)
    end

    subscribe(lcm, "BOX_ATLAS_STATE", on_robot_state, robot_state_t)
    @async while true
        handle(lcm)
    end
end

