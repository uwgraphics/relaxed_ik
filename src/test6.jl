a = [1,2,3,4,5]
b = ["a","b","c","d","e"]

using Random
using Knet

function shuffle_ins_and_outs(ins, outs)
    new_ins = []
    new_outs = []

    idxs = 1:length(ins)
    shuffled_idxs = shuffle(idxs)

    for i=1:length(shuffled_idxs)
        push!(new_ins, ins[shuffled_idxs[i]])
        push!(new_outs, outs[shuffled_idxs[i]])
    end

    return new_ins, new_outs
end

function get_batched_data(ins, outs, batch_size)
    batch = Knet.minibatch(ins, outs, batch_size)
    batches = []

    for (index, value) in enumerate(batch)
        push!(batches, value)
    end

    batched_data = []
    for batch_idx = 1:length(batches)
        push!(batched_data, [])
        for i = 1:length(batches[batch_idx][1])
            in = batches[batch_idx][1][i]
            out = batches[batch_idx][2][i]
            push!( batched_data[batch_idx], (in, out) )
        end
    end

    return batched_data
end

ins, outs = shuffle_ins_and_outs(a, b)
batched_data = get_batched_data(ins, outs, 2)

println(batched_data)
