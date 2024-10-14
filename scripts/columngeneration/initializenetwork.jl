


"""
Function to add a new time-space arc with start node, end node and arc profit
"""
function add_new_route(start_r::Int, end_r::Int, g_r::Float64)
    push!(rset, RouteSet(start_r, end_r, g_r, Set{Int}(), Set{Int}(), Set{Int}(), Int[], Vector{Tuple{Int64, Int64, Int64}}()))
end

#---------------------------------------------------------------------------------------#

"""
Function to create all time-space nodes and arcs
"""
function initializenetwork(W::Int)
    
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

    # ----------------
    # ii. Initialize time-space arcs r
    # ----------------

    for s_1 in 1:s # Start_and_End
        for s_2 in 1:s
            if (n_location_s[s_1] == 0 && n_location_s[s_2] != 0 && n_location_s[s_2] != TOTAL_Station + 1) # 230502
                if time_s[s_1] == time_s[s_2]
                    for _ in 1:W
                        add_new_route(s_1, s_2, -0.001)
                    end
                end
            end
            if (n_location_s[s_1] != 0 && n_location_s[s_1] != TOTAL_Station + 1 && n_location_s[s_2] == TOTAL_Station + 1) # 230502
                if time_s[s_1] + ceil(Bus_station_travel_time[n_location_s[s_1], n_location_s[s_2]] / unit) == time_s[s_2]
                    for _ in 1:W
                        add_new_route(s_1, s_2, -0.001)
                    end
                end
            end
            # If same location, then holding arc
            if n_location_s[s_1] == n_location_s[s_2] # Holding arcs
                if time_s[s_1] == time_s[s_2] - 1
                    for _ in 1:W
                        add_new_route(s_1, s_2, -0.001)
                    end
                end
            end
        end
    end
    r = length(rset)
    global timespace_route1 = 0
    println("Dummy and holding arcs = ", length(rset) - timespace_route1)
    global timespace_route2 = length(rset) - timespace_route1 
end
