function jvp(f, x, u)
    return FiniteDiff.finite_difference_derivative(t->f(x + t*u), 0.0)
end

### Task maps
function posture_task_map(θ, θ̇, prob::Problem)
    sys = prob.sys
    g = sys.θᵣ
    return θ - g
end

function lefthand_attractor_task_map(θ, θ̇, prob::Problem)
    sys = prob.sys
    x₉ = sys.g
    xh = left_hand_pose(θ, sys)
    return xh - x₉
end

function righthand_attractor_task_map(θ, θ̇, prob::Problem)
    sys = prob.sys
    x₉ = sys.g
    xh = right_hand_pose(θ, sys)
    return xh - x₉
end

function head_attractor_task_map(θ, θ̇, prob::Problem)
    sys = prob.sys
    x₉ = sys.g[1] 
    xh = head_pose(θ, sys)
    return xh - x₉
end

function waist_attractor_task_map(θ, θ̇, prob::Problem)
    sys = prob.sys
    x₉ = sys.g[3] 
    xh = head_pose(θ, sys)
    return xh - x₉
end

function dodge_task_map(θ, θ̇, prob::Problem)
    sys = prob.sys
    os = sys.obstacle_observables
    rs = sys.r
    chains = link_poses(θ, sys)
    xs = Float64[]; o = os[1]; r = 2*rs[1]
    lh = chains[5][end]; rh = chains[4][end]
    lẍ= chains[1][end]; rẍ= chains[2][end]
    hd = chains[3][end]; nk = chains[6][1]
    kp = [lh, rh, hd, nk]#[hd, lh, rh, lf, rf]
    for k in kp 
        Δ = (norm(k - o.val)/r)[1] - 1.0
        push!(xs, Δ)
    end 
    # @show xs
    return xs
end

### fabrics 
function posture_fabric(x, ẋ, prob::Problem)
    sys = prob.sys
    K = 1.6/3
    ϕ(x) = 0.5*x'*K*x
    δₓ = FiniteDiff.finite_difference_gradient(ϕ, x)
    M = I(length(x))
    ẍ= -K*δₓ
    return (M, ẍ)
end

function lefthand_attractor_fabric(x, ẋ, prob::Problem)
    sys = prob.sys
    K = 2*1.6
    ϕ(x) = 0.5*x'*K*x
    δₓ = FiniteDiff.finite_difference_gradient(ϕ, x)
    M = I(length(x))
    ẍ= -K*δₓ
    return (M, ẍ)
end

function righthand_attractor_fabric(x, ẋ, prob::Problem)
    sys = prob.sys
    K = 2*1.6
    ϕ(x) = 0.5*x'*K*x
    δₓ = FiniteDiff.finite_difference_gradient(ϕ, x)
    M = I(length(x))
    ẍ= -K*δₓ
    return (M, ẍ)
end

function head_attractor_fabric(x, ẋ, prob::Problem)
    sys = prob.sys
    K = 3*1.6
    ϕ(x) = 0.5*x'*K*x
    δₓ = FiniteDiff.finite_difference_gradient(ϕ, x)
    M = I(length(x))
    ẍ= -K*δₓ
    return (M, ẍ)
end

function waist_attractor_fabric(x, ẋ, prob::Problem)
    sys = prob.sys
    K = 3*1.6
    ϕ(x) = 0.5*x'*K*x
    δₓ = FiniteDiff.finite_difference_gradient(ϕ, x)
    M = I(length(x))
    ẍ= -K*δₓ
    return (M, ẍ)
end

function dodge_fabric(x, ẋ, prob::Problem)
    sys = prob.sys 
    K = 1.6 
    s = [v > sys.max_range ? 0.0 : 1.0 for v in x]  
    ϕ(σ) = (K/2) .* s .* (sys.max_range .- σ)./(sys.max_range .* σ).^2
    δₓ = FiniteDiff.finite_difference_jacobian(ϕ, x) 
    M = I(length(x))
    ẍ=-K*diag(δₓ)  
    return (M, ẍ)
end

### Solve  

function fabric_eval(x, ẋ, name::Symbol, prob::Problem, sys::PickleRick)
    M = nothing; ẍ = nothing 
    fabricname = Symbol(name, :_fabric)
    ϕ = eval(fabricname)
    M, ẍ = ϕ(x, ẋ, prob)
    return (M, ẍ)
end

function picklerick_fabric_solve(θ, θ̇ , prob::Problem, sys::PickleRick)
    xₛ = []; ẋₛ = []; cₛ = []
    Mₛ = []; ẍₛ = []; Jₛ = []
    for t in prob.tasks
        ψ = eval(Symbol(t, :_task_map))
        x = ψ(θ, θ̇,  prob) 
        ẋ = jvp(σ->ψ(σ, θ̇,  prob), θ, θ̇ ) 
        c = zero(x)
        J = FiniteDiff.finite_difference_jacobian(σ->ψ(σ, θ̇, prob), θ)
        M, ẍ = fabric_eval(x, ẋ, t, prob, sys)
        push!(xₛ, x); push!(ẋₛ, ẋ); push!(cₛ, c) 
        push!(Mₛ, M); push!(ẍₛ, ẍ); push!(Jₛ, J) 
    end   
    Mᵣ = sum([J' * M * J for (J, M) in zip(Jₛ, Mₛ)])
    fᵣ = sum([J' * M * (ẍ - c) for (J, M, ẍ, c) in zip(Jₛ, Mₛ, ẍₛ, cₛ)])
    Mᵣ = convert(Matrix{Float64}, Mᵣ)
    ẍ = pinv(Mᵣ) * fᵣ   
    return ẍ 
end