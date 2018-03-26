using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics
const rbd = RigidBodyDynamics
using RigidBodyDynamics: Bounds
using LCMCore
using BotCoreLCMTypes
using StaticArrays
using Rotations

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

atlas_world_frame = CartesianFrame3D("atlas_world")
add_frame!(root_body(robot), Transform3D(atlas_world_frame, default_frame(root_body(robot)), RotZ(π/2)))

state = MechanismState(robot)

vis = Visualizer()
mvis = MechanismVisualizer(robot, URDFVisuals(urdf), vis)
open(mvis.visualizer)
wait(mvis.visualizer)

configurations = Channel{Any}(32)

lcm = LCM()
function on_robot_state(channel::String, msg::robot_state_t)
    com_position = Point3D(atlas_world_frame, msg.pose.translation.x, msg.pose.translation.y, msg.pose.translation.z)
    
    world = default_frame(root_body(robot))
    pcom = relative_transform(state, atlas_world_frame, world) * com_position
    set_configuration!(state, findjoint(robot, "base_x"), pcom.v[1])
    set_configuration!(state, findjoint(robot, "base_z"), pcom.v[3])

    for (body, idxs, sign, θ) in (("lf", 7:9, 1, π), ("rf", 10:12, -1, π), ("lh", 1:3, 1, π/2), ("rh", 4:6, -1, π/2))
        position = Point3D(atlas_world_frame, msg.joint_position[idxs])
        rotation_joint = findjoint(robot, "core_to_$(body)_rotation")
        offset = relative_transform(state, atlas_world_frame, frame_before(rotation_joint)) * position
        set_configuration!(state, rotation_joint, θ + sign * atan2(offset.v[1], offset.v[3]))

        extension_joint = findjoint(robot, "core_to_$(body)_extension")
        offset = relative_transform(state, atlas_world_frame, frame_before(extension_joint)) * position
        set_configuration!(state, extension_joint, offset.v' * extension_joint.joint_type.axis)
    end
    put!(configurations, configuration(state))
end

@async while true
    q = take!(configurations)
    while isready(configurations)
        q = take!(configurations)
    end
    set_configuration!(mvis, q)
    sleep(1/30)
end

subscribe(lcm, "BOX_ATLAS_STATE", on_robot_state, robot_state_t)
while true
    handle(lcm)
end
