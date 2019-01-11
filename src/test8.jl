quit = false

for i=1:100000
    global quit
    if quit == true
        break
    end

    for j=1:101
        println("$i, $j")
        if i == 100 && j == 100
            println("$i, $j")
            println(":)")
            quit = true
            break
        end
    end
end

println("got here!")
