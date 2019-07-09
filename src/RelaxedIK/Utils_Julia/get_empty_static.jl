using StaticArrays

function get_empty_static_vector(n)
    if n == 2
        return Array{SArray{Tuple{2},Float64,1,2},1}()
    elseif n == 3
        return Array{SArray{Tuple{3},Float64,1,3},1}()
    elseif n == 4
        return Array{SArray{Tuple{4},Float64,1,4},1}()
    elseif n == 5
        return Array{SArray{Tuple{5},Float64,1,5},1}()
    elseif n == 6
        return Array{SArray{Tuple{6},Float64,1,6},1}()
    elseif n == 7
        return Array{SArray{Tuple{7},Float64,1,7},1}()
    elseif n == 8
        return Array{SArray{Tuple{8},Float64,1,8},1}()
    elseif n == 9
        return Array{SArray{Tuple{9},Float64,1,9},1}()
    elseif n == 10
        return Array{SArray{Tuple{10},Float64,1,10},1}()
    elseif n == 11
        return Array{SArray{Tuple{11},Float64,1,11},1}()
    elseif n == 12
        return Array{SArray{Tuple{12},Float64,1,12},1}()
    elseif n == 13
        return Array{SArray{Tuple{13},Float64,1,13},1}()
    elseif n == 14
        return Array{SArray{Tuple{14},Float64,1,14},1}()
    elseif n == 15
        return Array{SArray{Tuple{15},Float64,1,15},1}()
    elseif n == 16
        return Array{SArray{Tuple{16},Float64,1,16},1}()
    elseif n == 17
        return Array{SArray{Tuple{17},Float64,1,17},1}()
    elseif n == 18
        return Array{SArray{Tuple{18},Float64,1,18},1}()
    elseif n == 19
        return Array{SArray{Tuple{19},Float64,1,19},1}()
    elseif n == 20
        return Array{SArray{Tuple{20},Float64,1,20},1}()
    elseif n == 21
        return Array{SArray{Tuple{21},Float64,1,21},1}()
    elseif n == 22
        return Array{SArray{Tuple{22},Float64,1,22},1}()
    elseif n == 23
        return Array{SArray{Tuple{23},Float64,1,23},1}()
    elseif n == 24
        return Array{SArray{Tuple{24},Float64,1,24},1}()
    elseif n == 25
        return Array{SArray{Tuple{25},Float64,1,25},1}()
    elseif n == 26
        return Array{SArray{Tuple{26},Float64,1,26},1}()
    elseif n == 27
        return Array{SArray{Tuple{27},Float64,1,27},1}()
    elseif n == 28
        return Array{SArray{Tuple{28},Float64,1,28},1}()
    elseif n == 29
        return Array{SArray{Tuple{29},Float64,1,29},1}()
    elseif n == 30
        return Array{SArray{Tuple{30},Float64,1,30},1}()
    end
end

function write_get_empty_static_vector_function_string(n)
    s = "\n"
    s *= "function get_empty_static_vector(n)\n"
    s *= "\tif n == 2\n"
    s *= "\t\treturn Array{SArray{Tuple{2},Float64,1,2},1}()\n"
    if n > 2
        for i = 3:n
            s *= "\telseif n == $i\n"
            s *= "\t\treturn Array{SArray{Tuple{$i},Float64,1,$i},1}()\n"
        end
    end
    s *= "\tend\n"
    s *= "end"
    return s
end
