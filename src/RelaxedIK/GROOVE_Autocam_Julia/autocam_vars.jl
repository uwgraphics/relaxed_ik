
mutable struct Autocam_vars
    search_direction
    occlusion_score
    camera_goal_position
    visual_target_position
    previous_camera_location
    distance_to_target
    goal_distance_to_target
    outer_cone_max
end

function Autocam_vars()
    return Autocam_vars([1.,0.,0.], 0., [0.,0.,0.], [0.,0.,0.], [0.,0.,0.], 0., 0.6, 1.0)
end
