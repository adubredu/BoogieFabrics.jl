module BoogieFabrics

using GLMakie
using DataStructures: CircularBuffer
using FiniteDiff 
using LinearAlgebra
using StaticArrays
using Colors
  
include("types.jl")
include("picklerick/types.jl")
include("picklerick/kinematics.jl")
include("picklerick/dynamics.jl")
include("picklerick/simulate.jl")
include("picklerick/visualize.jl")
include("picklerick/fabric.jl") 

export visualize_system!,
       step!,
       Problem

export PointMass, 
       move_obstacles!,
       picklerick_fabric_solve

export PickleRick

end
