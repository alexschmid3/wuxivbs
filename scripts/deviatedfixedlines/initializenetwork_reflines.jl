


"""
Function to add a new time-space arc with start node, end node and arc profit
"""
function add_new_route(start_r::Int, end_r::Int, g_r::Float64, ln::Int)
    push!(rset[ln], RouteSet(start_r, end_r, g_r, Set{Int}(), Set{Int}(), Set{Int}(), Int[], Vector{Tuple{Int64, Int64, Int64}}()))
end

#---------------------------------------------------------------------------------------#

"""
Function to create all time-space nodes and arcs
"""
function initializenetwork_reflines(W::Int)
    
    # ----------------
    # i. Create all time-space nodes s
    # ----------------

    earliest_time = floor(Int, 0 / unit)
    latest_time = ceil(T_0 / unit)
    
    for stopXX = 1:TOTAL_Station
        for t = earliest_time:latest_time
            global s += 1
            n_location_s[s] = stopXX
            time_s[s] = Int(t)
            s_st[stopXX, time_s[s] + 1] = s
            #println(" s = ", s, " n_location_s[s] = ", n_location_s[s], " time_s[s] = ", time_s[s])
        end
    end

    # Dummy depot for everything
    global s += 1
    n_location_s[s] = 0
    time_s[s] = 0

    global s += 1
    n_location_s[s] = TOTAL_Station + 1
    time_s[s] = ceil(T_0 / unit)
    
    for jj = 1:n
        order = oset[jj]
        F_start_j[jj] = typemax(Int)
        F_end_j[jj] = typemin(Int)

        # 3-3 in the code, it was 0:TOTAL_Station + 1
        for stopXX in Hub_Possiblematch_station[jj] # DIFFERNECE 
            earliest_time1 = floor((order.eb + shortest_arrive_transfer_duration_jstation[jj, stopXX]) / unit)
            latest_time2 = ceil((order.dd - shortest_arrive_destination_duration_jstation[jj, stopXX]) / unit)
            
            earliest_time1 = Int(earliest_time1)
            latest_time2 = Int(latest_time2)

            for t = earliest_time1:latest_time2
                global f += 1
                n_location_f[f] = jj
                time_f[f] = t
                hub_f[f] = stopXX
                f_jht[jj, stopXX, t] = f

                #println(" f = ", f, " n_location_f[f] = ", n_location_f[f], " time_f[f] = ", time_f[f], " hub_f[f] = ", hub_f[f])
                
                if F_start_j[jj] > f
                    F_start_j[jj] = f
                end

                if F_end_j[jj] < f
                    F_end_j[jj] = f
                end
            end
        end
    end

    for ss in 1:s
        nodelookup[n_location_s[ss], time_s[ss]] = ss
    end

    # ----------------
    # ii. Initialize time-space arcs r
    # ----------------

    for ln in 1:numlines
        for s_1 in 1:s # Start_and_End
            s1_loc, s1_time = n_location_s[s_1], time_s[s_1]
            if s1_loc in coverage[ln] || s1_loc in [0, TOTAL_Station+1]
                #Arcs from dummy origin
                if (s1_loc == 0)
                    for s2_loc in coverage[ln]
                        try
                            s_2 = nodelookup[s2_loc, s1_time]
                            add_new_route(s_1, s_2, -0.001, ln)
                        catch
                            1+1
                        end
                    end
                end

                #Arcs to dummy destination
                if (s1_loc != 0 && s1_loc != TOTAL_Station + 1)
                    try
                        s_2 = nodelookup[TOTAL_Station + 1, s1_time + ceil(Bus_station_travel_time[s1_loc, TOTAL_Station + 1] / unit)]
                        add_new_route(s_1, s_2, -0.001, ln)
                    catch
                        1+1
                    end
                end

                #Holding arcs
                try
                    s_2 = nodelookup[s1_loc, s1_time+1]
                    add_new_route(s_1, s_2, -0.001, ln)
                catch
                    1+1
                end
            end
        end
    end

    r = sum(length(rset[ln]) for ln in 1:numlines)
    println("r after holding arcs is $r")
    for ln in 1:numlines
        timespace_route1[ln] = 0
        timespace_route2[ln] = length(rset[ln]) - timespace_route1[ln]
    end
end
