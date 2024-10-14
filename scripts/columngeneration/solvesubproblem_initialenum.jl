
function Procedure_DP_for_bus_jj_cg_initenum(lset::Vector{LabelSet}, Start_node::Int, Start_time::Int, index_start::Int, e::Int, ll::Int, bus_jj::Int, printstatements::Int, localrset::Vector{RouteSet})
    se = lset[e]
    bus_ii = se.BUS_station_l

    #bus_ii - station we're at
    #bus_jj - station we're headed to

    #If (has no extensions & at original station) || (has extensions)
    if (se.S_l == 0 && bus_jj == Start_node) || se.S_l > 0
        #If we are allowed to visit bus_jj
        if !(bus_jj in se.BUS_station_forbidden)           
            #If (ii != jj and no more passengers to dropoff at ii) || (ii == jj and jj not in start_pickup)
            if (bus_jj != bus_ii && !(bus_ii in se.BUS_station_continue_dropoff)) || (bus_jj == bus_ii && !(bus_jj in se.BUS_station_start_pickup))
                #for order j such that jj is a possible dropoff destination
                for j in drelate_j_station[bus_jj]
                    # if j has been picked up, but not dropped off
                    if j - n in se.preset1_ll 
                        # if same station like 1,3 = 3,1, then remove dup
                        if (se.n_l < j && bus_jj == bus_ii) || bus_jj != bus_ii #check that we dropoff passengers "in order", to avoid enumerating permutations of dropping off multiple passengers at the same stop
                            if bus_jj in Possiblematch_station_d[j - n] || (bus_jj in Hub_Possiblematch_station[j - n] && !(j - n in se.preset1transfer_ll)) # Conduct transfer dropoff: no transfer pickup previous
                                ll += 1                                          #Next level of depth
                                sl = get_New_LabelJ(se, e, bus_jj, j)            #New label 
                                if check_New_LabelJ(sl, bus_jj, j)
                                    lset[ll] = sl
                                    localrset = Procedure_DP_for_generating_arcs_cg_initenum(lset, Start_node, Start_time, index_start, ll, printstatements, localrset)
                                end
                                ll -= 1
                            end
                        end
                    end
                end
            end
            # upupupup
            if !(bus_ii in se.BUS_station_continue_dropoff) # All possible dropoffs have been put down
                for j in prelate_j_station[bus_jj]
                    order = oset[j]
                    if (se.n_l < j && bus_jj == bus_ii && bus_jj in se.BUS_station_start_pickup) || !(bus_jj in se.BUS_station_start_pickup) # if it has been done or not done yet
                        if (bus_jj in Possiblematch_station_o[j] && order.eb < se.T_l + Bus_station_travel_time[bus_ii, bus_jj] && order.dd - shortest_direct_arrive_destination_duration_jstation[j, bus_jj] > se.T_l + Bus_station_travel_time[bus_ii, bus_jj]) || (bus_jj in Hub_Possiblematch_station[j] && order.eb + shortest_arrive_transfer_duration_jstation[j, bus_jj] < se.T_l + Bus_station_travel_time[bus_ii, bus_jj] && order.dd - shortest_arrive_destination_duration_jstation[j, bus_jj] > se.T_l + Bus_station_travel_time[bus_ii, bus_jj])
                            if !(j in se.preset2_ll) && se.Q_l + order.q <= Q # Customer can be picked up
                                ll += 1
                                sl = get_New_LabelJ(se, e, bus_jj, j)
                                if check_New_LabelJ(sl, bus_jj, j)
                                    lset[ll] = sl
                                    localrset = Procedure_DP_for_generating_arcs_cg_initenum(lset, Start_node, Start_time, index_start, ll, printstatements, localrset)
                                end
                                ll -= 1
                            end
                        end
                    end
                end
            end
        end
    end

    return localrset
end


#---------------------------------------------------------------------------------------#

"""
Function to add a new time-space arc
"""
function Procedure_DP_for_check_arc_cg_initenum(lset::Vector{LabelSet}, Start_time::Int, index_start::Int, ll::Int, allstations::Vector{Int}, alltimes::Vector{Float64}, allorders::Vector{Tuple{Int64,Int64,Base.Int64}}, localrset::Vector{RouteSet})
    se = lset[ll]

    stopXX = se.BUS_station_l
    tt = Int(ceil(se.T_l / unit))
    ss = s_st[stopXX, tt + 1]

    start_r = index_start
    end_r = ss
    g_r = 0.0
    preseta_r = Set{Int}()
    presetb1_r = Set{Int}()
    presetb2_r = Set{Int}()
    nodeset_r = allstations
    customer_pudo_r = allorders

    #Backtrack through the levels (i.e. traverse route backwards) to find passengers and transfers
    for pl in ll:-1:1
        pre_vertex = lset[pl].n_l
        if pre_vertex <= n && pre_vertex > 0 
            if lset[pl].BUS_station_l in Hub_Possiblematch_station[pre_vertex]
                ff = f_jht[pre_vertex, lset[pl].BUS_station_l, Int(ceil(lset[pl].T_l / unit))] # !!! Added to be int on 4-5
                push!(presetb2_r, ff)
            end
        end
        if pre_vertex > n && pre_vertex < 2 * n + 1 
            if lset[pl].BUS_station_l in Possiblematch_station_d[pre_vertex - n]
                if !(pre_vertex - n in lset[pl].preset1transfer_ll)
                    push!(preseta_r, pre_vertex - n)
                    g_r += oset[pre_vertex - n].g
                end
            end

            if lset[pl].BUS_station_l in Hub_Possiblematch_station[pre_vertex - n]
                ff = f_jht[(pre_vertex - n), lset[pl].BUS_station_l, Int(ceil(lset[pl].T_l / unit))] # !!! Added to be int on 4-5
                push!(presetb1_r, ff)
                g_r += oset[pre_vertex - n].g # -0.1
            end
        end
    end
    g_r = g_r + ((-se.T_l + Start_time) / unit) * unit * cost

    push!(localrset, RouteSet(start_r, end_r, g_r, preseta_r, presetb1_r, presetb2_r, nodeset_r, customer_pudo_r))

    return localrset
end

#---------------------------------------------------------------------------------------#

"""
Function to produce time-space arcs with given start node and time (Main Part of dynamic programming)

### input
* `lset` - label setting set
* `Start_node` - station id
* `Start_time` - time
* `index_start` - time-space node id
* `ll` - label setting recursion depth
"""

function Procedure_DP_for_generating_arcs_cg_initenum(lset::Vector{LabelSet}, Start_node::Int, Start_time::Int, index_start::Int, ll::Int, printstatements::Int, localrset::Vector{RouteSet})
    
    proc_begtime = now()
    # if depth equals 1, need to initialize the label set
    if ll == 1
        lset[1] = get_Init_LabelSet(Start_node, Start_time)
    end

    e = ll # record the depth
    se = lset[e] # get currnet label status
    bus_ii = se.BUS_station_l # get current station
    
    # if not the initial status and no passengers here, then add the arc and end the recursio
    if se.Q_l == 0 && se.S_l > 0
        allstations, alltimes, allorders = getfullroute(lset, ll)
        localrset = Procedure_DP_for_check_arc_cg_initenum(lset, Start_time, index_start, ll, allstations, alltimes, allorders, localrset)
        return localrset
    end                                                                                             
 
    # set station status before DP extension (i.e. update se.BUS_station_continue_dropoff and se.BUS_station_forbidden)
    Procedure_DP_set_station(lset, ll)
    
    # recursively extend the label for each bus station, be careful we need to start from bus_ii, the current station
    for bus_jj in union(bus_ii:TOTAL_Station, bus_ii - 1:-1:1)
        localrset = Procedure_DP_for_bus_jj_cg_initenum(lset, Start_node, Start_time, index_start, e, ll, bus_jj, printstatements, localrset)
    end

    proc_chunk4_time = now()

    total_interval = proc_chunk4_time - proc_begtime
    #if (total_interval.value / 1000) >= 30
    #    println("Total time is $(total_interval.value / 1000)")
    #end

    return localrset
end

#---------------------------------------------------------------------------------------#

function enumerateroutes()

    lset = Vector{LabelSet}(undef, L)

    Time_longest_dp = -1
    begin_dp_total = now()
    end_dp_total = now()

    sp_rset = Vector{RouteSet}()

    num_dp = 0
    enumstarttime = time()
    println("********************* Begin route enumeration *********************")
    println("Dummy and holding arcs = ", length(rset))
    for s_1 in 2:s
        if n_location_s[s_1] != TOTAL_Station + 1 && n_location_s[s_1] != 0 
            num_dp += 1
            begin_dp = now()

            localrset = Vector{RouteSet}()

            r = length(localrset)
            #println("r before procedure is: $r")
           
            stm=time()
            #Find all negative reduced cost routes
            localrset = Procedure_DP_for_generating_arcs_cg_initenum(lset, n_location_s[s_1], time_s[s_1] * unit, s_1, 1, 0, localrset)
            ttm = time()-stm
            #if ttm > 0.1
            #    println("s_1 = $s_1 --> ", ttm)
            #end
            r = length(localrset)
            #println("r after procedure is: $r for $s_1")

            #Check for new best route
            for rr in localrset
                push!(sp_rset, rr)
            end
            
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
    println("Movement subpaths = ", length(rset) - timespace_route2)
    println("********************** End route enumeration **********************")
        
    end_dp_total = now()
    interval_dp_total = end_dp_total - begin_dp_total
    println("Total >> DP_total DONE! The Totmin is $(interval_dp_total.value / (1000 * 60)) or Totsec: $(interval_dp_total.value / (1000))")
    println("Parallelize >> DP_total DONE! The Totmin is $(Time_longest_dp / 60) or Totsec: $Time_longest_dp")
    
    return sp_rset

end

#---------------------------------------------------------------------------------------#

function solvesubproblem_initialenum(dualinfo, localrset)

    bestroute = nothingroute
    bestrc = 0

    for rr in localrset
        routereducedcost = (rr.g_r 
        + dualinfo.alpha1[rr.start_r] - dualinfo.alpha1[rr.end_r]
        - sum(dualinfo.beta[j] for j in rr.preseta_r; init=0) - sum(dualinfo.beta[j] for j in 1:n if is_noempty_intersect(rr.presetb1_r, fjset[j]); init=0) 
        - sum(dualinfo.gamma[j] for j in 1:n if is_noempty_intersect(rr.presetb1_r, fjset[j]); init=0) + sum(dualinfo.gamma[j] for j in 1:n if is_noempty_intersect(rr.presetb2_r, fjset[j]) ; init=0)
        - sum(dualinfo.delta[ff] for ff in rr.presetb1_r; init=0) + sum(dualinfo.delta[ff] for ff in 1:f if is_noempty_intersect(rr.presetb2_r, ffset[ff]); init=0) )

        #Check if route has negative reduced cost
        if routereducedcost > bestrc + 0.0001
            bestroute = rr
            bestrc = routereducedcost
        end
    end

    tempbestroute = TempRouteSet(bestroute.start_r,bestroute.end_r,bestroute.g_r,bestroute.preseta_r,bestroute.presetb1_r,bestroute.presetb2_r,bestroute.nodepath_r,bestroute.customer_pudo_r,bestrc)

    return tempbestroute

end