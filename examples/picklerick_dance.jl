using Revise
using BoogieFabrics

θ = [6.0, 0.0]
θ̇ = [0.0, 0.0]
Obstacles = [[-4.0, 3.5]]
Obstacle_radii = [1.0]
goal = [-2.0, 4.0]

init_joint_positions = zeros(10)

sys = PickleRick(Obstacles, 0.5 .* Obstacle_radii,
                init_joint_positions,
                zero(init_joint_positions),
                goal)
 
sys.show_contacts = false
sys.dynamic = true
sys.show_obstacle = true  
sys.show_goal = true
ax, fig = visualize_system!(sys)

tasks = [:posture, :lefthand_attractor, :dodge]
prob = Problem(tasks, sys)
horizon = 10000

θ = init_joint_positions
θ̇ = zero(init_joint_positions)

for i=1:horizon 
    global θ, θ̇ 
    τ = picklerick_fabric_solve(θ, θ̇ , prob, sys)
    step!(τ, prob, sys)
    θ = prob.sys.θ; θ̇ = prob.sys.θ̇
    sleep(prob.sys.Δt/horizon)
end