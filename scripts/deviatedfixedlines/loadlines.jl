
referencelines = Dict(
    1 => [2, 40, 9, 11, 7, 2],             #blue
    2 => [10, 3, 2, 7, 51, 12, 10],     #orange
    3 => [31, 38, 20, 1, 3, 26, 31],    #pink
    4 => [20, 63, 62, 59, 31, 20],      #purple
    5 => [9, 11, 7, 2, 40, 9],             #blue
    6 => [3, 26, 31,38, 20, 1, 3],    #pink
    7 => [2, 7, 51, 12, 10, 3, 2],     #orange
    8 => [59, 31, 20, 63, 62, 59]       #purple
) 

vehiclestartlocs = [referencelines[w][1] for w in 1:W]