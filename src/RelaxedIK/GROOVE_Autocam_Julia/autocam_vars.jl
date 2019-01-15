
mutable struct Autocam_vars
    search_direction
    occlusion_score
    camera_goal_position
    visual_target_position
    previous_camera_location
    distance_to_target
    goal_distance_to_target
end

function Autocam_vars()
    return Autocam_vars([1.,0.,0.], 0., [0.,0.,0.], [0.,0.,0.], [0.,0.,0.], 0., 0.6)
end
