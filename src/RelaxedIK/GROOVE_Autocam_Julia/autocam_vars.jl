
mutable struct Autocam_vars
    search_direction
    occlusion_score
    visual_target_position
    previous_camera_location
    distance_to_target
    goal_distance_to_target
    too_close
end

function Autocam_vars()
    return Autocam_vars([1.,0.,0.], 0., [0.,.1,.5], [0.,0.,0.], 0., 0.6, false)
end
