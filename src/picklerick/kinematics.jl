# foot as reference
function link_poses(θ, sys::PickleRick)
    l1 = sys.l1
    l2 = sys.l2
    l3 = sys.l3
    l4 = sys.l4
    l5 = sys.l5
    l6 = sys.l6
    l7 = sys.l7
    l8 = sys.l8
    l9 = sys.l9
    l10 = sys.l10
    l0 = sys.l0
    w = sys.w 
    x8 = -w
    y8 = 0.0
    x10 = w 
    y10 = 0.0
    x7 = x8 + l8 * cos(θ[8])
    y7 = y8 + l8 * sin(θ[8])
    x9 = x10 + l10 * cos(θ[10])
    y9 = y10 + l10 * sin(θ[10]) 
    x6 = 0.5*(x7 + l7 * cos(θ[7]) + x9 + l9 * cos(θ[9]))
    y6 = 0.5*(y7 + l7 * sin(θ[7]) + y9 + l9 * sin(θ[9]))
    x1 = x6 + l1 * cos(θ[6])
    y1 = y6 + l1 * sin(θ[6])
    x2 = x1 - l2 * sin(θ[2])
    y2 = y1 + l2 * cos(θ[2])
    x3 = x2 - l3 * sin(θ[3])
    y3 = y2 + l3 * cos(θ[3])
    x4 = x1 + l4 * sin(θ[4])
    y4 = y1 + l4 * cos(θ[4])
    x5 = x4 + l5 * sin(θ[5])
    y5 = y4 + l5 * cos(θ[5])
    x0 = x1 + l0 * cos(θ[1])
    y0 = y1 + l0 * sin(θ[1])
    xn = x1 + 0.5*l0 * cos(θ[1])
    yn = y1 + 0.5*l0 * sin(θ[1]) 

    return [[[x6, y6], [x7, y7], [x8, y8]], [[x6, y6], [x9, y9], [x10, y10]], 
    [[x6, y6], [x1, y1], [x0, y0]], [[x1, y1], [x4, y4], [x5, y5]], [[x1, y1], [x2, y2], [x3, y3]], [[xn, yn]]]
end

function left_hand_pose(θ, sys::PickleRick)
    poses = link_poses(θ, sys::PickleRick)
    return poses[5][end]
end

function right_hand_pose(θ, sys::PickleRick)
    poses = link_poses(θ, sys::PickleRick)
    return poses[4][end]
end

function left_foot_pose(θ, sys::PickleRick)
    poses = link_poses(θ, sys::PickleRick)
    return poses[1][end]
end

function right_foot_pose(θ, sys::PickleRick)
    poses = link_poses(θ, sys::PickleRick)
    return poses[2][end]
end

function head_pose(θ, sys::PickleRick)
    poses = link_poses(θ, sys::PickleRick)
    return poses[3][end]
end

function neck_pose(θ, sys::PickleRick)
    poses = link_poses(θ, sys::PickleRick)
    return poses[6][end]
end

function waist_pose(θ, sys::PickleRick)
    poses = link_poses(θ, sys)
    return poses[5][1]
end