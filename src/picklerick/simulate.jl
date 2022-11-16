function step!(θ̈ ::Vector{Float64}, prob::Problem, sys::PickleRick)
    θ = sys.θ + θ̈ *sys.Δt
    θ̇ = θ̈   
    vellims = 10.0*ones(length(θ))
    θ̇  = clamp.(θ̇ , -vellims, vellims)
    
    chains = link_poses(θ, sys)
    sys.body_observables[1][] = [SVector(a[1], a[2]) for a in chains[1]]
    sys.body_observables[2][] = [SVector(a[1], a[2]) for a in chains[2]]
    sys.body_observables[3][] = [SVector(a[1], a[2]) for a in chains[3]]
    sys.body_observables[4][] = [SVector(a[1], a[2]) for a in chains[4]]
    sys.body_observables[5][] = [SVector(a[1], a[2]) for a in chains[5]]
    sys.body_observables[6][] = [SVector(chains[3][3][1], chains[3][3][2])]

    sys.θ = θ
    sys.θ̇ = θ̇

    if sys.dynamic
        for ob in sys.obstacle_observables
            val = collect(ob.val)
            val[1] -= 0.00625
            ob[] = SVector(val...)
            if ob[][1] < -10.0
                val[1] = 10
                ob[] = SVector(val...)
            end
        end
    end

    if sys.show_com
        com = compute_COM(θ, sys)
        sys.com_observable[] = SVector(com[1],com[2])
    end
end