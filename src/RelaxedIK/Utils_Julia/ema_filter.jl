
mutable struct EMA_filter
    a
    window_size
    filtered_signal
end

function EMA_filter(init_state; a = 0.999, window_size = 26)
    if a > 1.0 || a < 0.0
        throw(ArgumentError("a must be between 0 and 1 in EMA filter"))
    end

    filtered_signal = []
    for i = 1:window_size+1
        push!(filtered_signal, init_state)
    end

    return EMA_filter(a, window_size, filtered_signal)
end

function filter_signal(EMA_filter, state)
    weights = zeros(EMA_filter.window_size)
    filtered_state = zeros(length(state))

    sum = 0.0
    for i = 1:EMA_filter.window_size
        weights[i] = EMA_filter.a*(1.0 - EMA_filter.a)^(i-1)
        if i == 1
            filtered_state = weights[i]*state
        else
            filtered_state = filtered_state + weights[i]*EMA_filter.filtered_signal[end-(i)]
        end
        sum += weights[i]
    end

    push!(EMA_filter.filtered_signal, filtered_state)
    return filtered_state
end
