using Interpolations

function get_linear_interp(pts)
    # pts can be a list of any dimensional points
    xs = range(0,stop=1,length=length(pts))
    interp_linear = LinearInterpolation(xs, pts)
    return interp_linear
end
