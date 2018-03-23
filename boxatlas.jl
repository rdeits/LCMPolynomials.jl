using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics
using LCMCore
using BotCoreLCMTypes

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
attach!(robot, base, urdf_mech)

mvis = MechanismVisualizer(robot, URDFVisuals(urdf))
open(mvis)
wait(mvis)

lcm = LCM()
function on_robot_state(channel::String, msg::robot_state_t)
    


