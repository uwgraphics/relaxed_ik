using DSP


function filter_signal(filter_obj, signal)
    # filter_obj must be a DSP filter obj
    # ex: digitalfilter(DSP.Filters.Lowpass(0.2), DSP.Butterworth(2))
    # signal can be any vector of floats or vectors
    if typeof(signal[1]) == Float64
        return filtfilt(filter_obj, signal)
    else
        filtered_signal = []
        for i = 1:length(signal[1])
            curr_coords = []
            for j = 1:length(signal)
                if i == 1
                    push!(filtered_signal, zeros(length(signal[1] )   )   )
                end
                push!(curr_coords, signal[j][i])
            end

            filtered_coords = filtfilt(filter_obj, curr_coords)
            for j = 1:length(filtered_coords)
                filtered_signal[j][i] = filtered_coords[j]
            end
        end
    end
    return filtered_signal
end
