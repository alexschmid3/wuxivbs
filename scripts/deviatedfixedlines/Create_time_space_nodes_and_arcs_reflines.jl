
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
function Create_time_space_nodes_and_arcs_reflines(W::Int)
    
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
        ord = oset[jj]
        F_start_j[jj] = typemax(Int)
        F_end_j[jj] = typemin(Int)

        # 3-3 in the code, it was 0:TOTAL_Station + 1
        for stopXX in Hub_Possiblematch_station[jj] # DIFFERNECE 
            earliest_time1 = floor((ord.eb + shortest_arrive_transfer_duration_jstation[jj, stopXX]) / unit)
            latest_time2 = ceil((ord.dd - shortest_arrive_destination_duration_jstation[jj, stopXX]) / unit)
            
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
    # ii. Create all time-space arcs r, for each line
    # ----------------

    enumtime = 0
    for ln in 1:numlines
        lset = Vector{LabelSet3}(undef, L)
        
        Time_longest_dp = -1
        begin_dp_total = now()
        end_dp_total = now()

        num_dp = 0
        println("r is: ", length(rset[ln]))
        enumstarttime = time()
        println("Beginning the thing (s = $s)")
        for s_1 in 2:s
            if (n_location_s[s_1] in coverage[ln]) && n_location_s[s_1] != TOTAL_Station + 1 && n_location_s[s_1] != 0 
                num_dp += 1
                begin_dp = now()

                r = length(rset[ln])
                #println("r before procedure is: $r")
            
                stm=time()
                Procedure_DP_for_generating_arcs(ln, lset, n_location_s[s_1], time_s[s_1] * unit, s_1, 1, 0)
                ttm = time()-stm
                if ttm > 0.5
                    println("s_1 = $s_1 --> ", ttm)
                end
                r = length(rset[ln])
                #println("r after procedure is: $r")
                
                end_dp = now()
                interval_dp = end_dp - begin_dp

                #println("*******************************************************************") # ??
                #println("$num_dp DP DONE! The Totmin is $(interval_dp.value / (1000 * 60)) or Totsec: $(interval_dp.value / 1000)") # ??

                if (interval_dp.value / 1000) > Time_longest_dp
                    Time_longest_dp = (interval_dp.value / 1000) # !!!! Note sure here
                    #println("Updating Time_longest_dp ~~~~~~~~~~~~~~~~~~~~~~~~~~ to: ", Time_longest_dp) # ??
                end

                #println("r is: $r for s_1 = $s_1") 
            end
            s_1 += 1
        end
        println("Ending the thing")
        enumtime += time() - enumstarttime
        println("Basic enumeration = ", enumtime)
        println("r is: ", length(rset[ln]))
        
        println("num_dp = $num_dp")
            
        end_dp_total = now()
        interval_dp_total = end_dp_total - begin_dp_total
        println("*******************************************************************")
        println("TotalTotalTotalTotal >> DP_total DONE! The Totmin is $(interval_dp_total.value / (1000 * 60)) or Totsec: $(interval_dp_total.value / (1000))")
        println("*******************************************************************")
        println("Parallelize >> DP_total DONE! The Totmin is $(Time_longest_dp / 60) or Totsec: $Time_longest_dp")

    end

    # ======================================================================================================== 
    #global timespace_route1
    for ln in 1:numlines
        timespace_route1[ln] = length(rset[ln])
    end
    println("r is: ", sum(length(rset[ln]) for ln in 1:numlines))
    #println("timespace_route1 = $timespace_route1")
    println("s = $s")

    for ln in 1:numlines
        for s_1 in 1:s # Start_and_End
            s1_loc, s1_time = n_location_s[s_1], time_s[s_1]
            if s1_loc in coverage[ln] || s1_loc in [0, TOTAL_Station+1]
                #for s_2 in 1:s

                    #=if (s1_loc == 0 && n_location_s[s_2] != 0 && n_location_s[s_2] != TOTAL_Station + 1) & (n_location_s[s_2] in coverage[ln] || n_location_s[s_2] in [0, TOTAL_Station+1])# 230502
                        if s1_time == time_s[s_2]
                            for _ in 1:W
                                add_new_route(s_1, s_2, -0.001, ln)
                            end
                        end
                    end=#

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

                    #=if (s1_loc != 0 && s1_loc != TOTAL_Station + 1 && n_location_s[s_2] == TOTAL_Station + 1) # 230502
                        if s1_time + ceil(Bus_station_travel_time[s1_loc, n_location_s[s_2]] / unit) == time_s[s_2]
                            for _ in 1:W
                                add_new_route(s_1, s_2, -0.001, ln)
                            end
                        end
                    end=#

                #Add arcs to dummy destination
                if (s1_loc != 0 && s1_loc != TOTAL_Station + 1)
                    try
                        s_2 = nodelookup[TOTAL_Station + 1, s1_time + ceil(Bus_station_travel_time[s1_loc, TOTAL_Station + 1] / unit)]
                        add_new_route(s_1, s_2, -0.001, ln)
                    catch
                        1+1
                    end
                end

                    # If same location, then holding arc
                    #=if s1_loc == n_location_s[s_2] # Holding arcs
                        if s1_time == time_s[s_2] - 1
                            for _ in 1:W
                                add_new_route(s_1, s_2, -0.001, ln)
                            end
                        end
                    end=#

                #Add holding arcs
                try
                    s_2 = nodelookup[s1_loc, s1_time+1]
                    #for _ in 1:W
                    add_new_route(s_1, s_2, -0.001, ln)
                    #end
                catch
                    1+1
                end
                #end
            end
        end
    end
    r = sum(length(rset[ln]) for ln in 1:numlines)
    println("r after holding arcs is $r")
    for ln in 1:numlines
        timespace_route2[ln] = length(rset[ln]) - timespace_route1[ln]
    end

    return enumtime
end