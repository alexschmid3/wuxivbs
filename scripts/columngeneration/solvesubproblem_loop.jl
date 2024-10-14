
function get_Init_LabelSet2(Start_node::Int, Start_time::Int)
    n_l = -100
    T_l = Start_time
    S_l= 0
    Q_l = 0
    prel_l = -1
    Dwell_l = 1
    Transfer_l = 0
    BUS_station_l = Start_node
    
    preset1_ll = Set{Int}()
    preset2_ll = Set{Int}()
    preset1transfer_ll = Set{Int}()
    BUS_station_continue_dropoff = Set{Int}()
    BUS_station_start_pickup = Set{Int}()
    BUS_station_forbidden = Set{Int}()

    prevlist_l = Int[]
    push!(prevlist_l, 1)

    return LabelSet2(n_l, T_l, S_l, Q_l, prel_l, Dwell_l, Transfer_l, BUS_station_l, preset1_ll, preset2_ll, preset1transfer_ll, BUS_station_continue_dropoff, BUS_station_start_pickup, BUS_station_forbidden, prevlist_l)

end


#---------------------------------------------------------------------------------------#

"""
Function to get new label to next bus station
"""
function get_New_LabelJ2(se::LabelSet2, e::Int, bus_jj::Int, j::Int, newindex::Int)
    bus_ii = se.BUS_station_l # current bus station

    n_l = j
    T_l = se.T_l + Bus_station_travel_time[bus_ii, bus_jj]
    if j > n
        Q_l = se.Q_l - oset[j - n].q
    else
        Q_l = se.Q_l + oset[j].q
    end

    S_l = se.S_l + 1
    Dwell_l = se.Dwell_l
    Transfer_l = se.Transfer_l
    if bus_ii != bus_jj
        Dwell_l += 1
        T_l += Dwell_time
    end
    prel_l = e
    
    BUS_station_l = bus_jj

    BUS_station_continue_dropoff = copy(se.BUS_station_continue_dropoff)
    BUS_station_start_pickup = copy(se.BUS_station_start_pickup)
    BUS_station_forbidden = Set{Int}()

    prevlist_l = copy(se.prevlist_l)
    push!(prevlist_l, newindex)

    if j > n
        if bus_jj in Hub_Possiblematch_station[j - n] # if is a hub transfer drop
            Transfer_l += 1
        end
        preset1_ll = setdiff(se.preset1_ll, Set(j - n))
        preset1transfer_ll = se.preset1transfer_ll
        preset2_ll = se.preset2_ll
        if bus_jj != bus_ii # Due to the possibility of repeatedly visiting a this_station
            delete!(BUS_station_start_pickup, bus_jj) # still need to visit
        end
        # currnet stop is visited, then we need mark it, and drop all can be dropped
        push!(BUS_station_continue_dropoff, bus_jj)
        return LabelSet2(n_l, T_l, S_l, Q_l, prel_l, Dwell_l, Transfer_l, BUS_station_l, preset1_ll, preset2_ll, preset1transfer_ll, BUS_station_continue_dropoff, BUS_station_start_pickup, BUS_station_forbidden, prevlist_l)
    else
        if bus_jj in Hub_Possiblematch_station[j] # if it is a hub transfer drop
            Transfer_l += 1
        end
        preset1_ll = union(se.preset1_ll, Set(j))
        preset2_ll = union(se.preset2_ll, Set(j))
        preset1transfer_ll = se.preset1transfer_ll
        if bus_jj in Hub_Possiblematch_station[j] && !(j in se.preset1transfer_ll)
            preset1transfer_ll = union(preset1transfer_ll, Set(j)) # Mark this j is a transfer pickup
        end
        push!(BUS_station_continue_dropoff, bus_jj)
        push!(BUS_station_start_pickup, bus_jj)
        return LabelSet2(n_l, T_l, S_l, Q_l, prel_l, Dwell_l, Transfer_l, BUS_station_l, preset1_ll, preset2_ll, preset1transfer_ll, BUS_station_continue_dropoff, BUS_station_start_pickup, BUS_station_forbidden, prevlist_l)
    end

end


#---------------------------------------------------------------------------------------#

function check_New_LabelJ2(se::LabelSet2, bus_jj::Int, j::Int)
    if j > n
        order = oset[j - n]
        # ========================= Feasibility check 1
        if se.T_l > order.dd || se.T_l > T_0 # Time check
            return false 
        end

        if bus_jj in Hub_Possiblematch_station[j - n] # 如果是个hub transfer drop
            check_here = -1
            for bus_check in Possiblematch_station_d[j - n]
                if se.T_l + Bus_station_travel_time[bus_jj, bus_check] + Dwell_time <= order.dd && se.T_l + Bus_station_travel_time[bus_jj, bus_check] + Dwell_time <= T_0
                    check_here = 1
                    break
                end
            end
            if check_here == -1
                return false
            end
        end
    end

    # ========================= Feasibility check 3
    for here_j in 1:n # 如果去了这个站点,别的放不下去，也不行
        order = oset[here_j]
        if here_j in se.preset1_ll
            check_f = -1
            for check_bus_jj in Possiblematch_station_d[here_j]
                dwell_here = 0.0
                if check_bus_jj != bus_jj
                    dwell_here = Dwell_time
                end
                if se.T_l + Bus_station_travel_time[bus_jj, check_bus_jj] + dwell_here <= order.dd && se.T_l + Bus_station_travel_time[bus_jj, check_bus_jj] + dwell_here <= T_0
                    check_f = 1
                    break
                end
            end
            if check_f == -1
                return false
            end
        end
    end
    return true
end


#---------------------------------------------------------------------------------------#

function Procedure_DP_for_bus_jj_cg_loop(se::LabelSet2, lset::Vector{LabelSet2}, labellookup::Dict{Int,LabelSet2}, Start_node::Int, ll::Int, bus_jj::Int, localrset::Vector{TempRouteSet})

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
                                #ll += 1                                          #Next level of depth
                                sl = get_New_LabelJ2(se, ll, bus_jj, j, length(labellookup)+1)            #New label 
                                if check_New_LabelJ2(sl, bus_jj, j)
                                    pushfirst!(lset, sl)
                                    labellookup[length(labellookup)+1] = sl
                                    #lset[ll] = sl
                                    #localrset = Procedure_DP_for_generating_arcs_cg(lset, Start_node, Start_time, index_start, ll, printstatements, localrset, dualinfo)
                                end
                                #ll -= 1
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
                                #ll += 1
                                sl = get_New_LabelJ2(se, ll, bus_jj, j, length(labellookup)+1)
                                if check_New_LabelJ2(sl, bus_jj, j)
                                    pushfirst!(lset, sl)
                                    labellookup[length(labellookup)+1] = sl
                                    #lset[ll] = sl
                                    #localrset = Procedure_DP_for_generating_arcs_cg(lset, Start_node, Start_time, index_start, ll, printstatements, localrset, dualinfo)
                                end
                                #ll -= 1
                            end
                        end
                    end
                end
            end
        end
    end

    return lset, labellookup, localrset
end


#---------------------------------------------------------------------------------------#

"""
Function to add a new time-space arc
"""
function Procedure_DP_for_check_arc_cg_loop(se::LabelSet2, labellookup::Dict{Int, LabelSet2}, Start_time::Int, index_start::Int, ll::Int, allstations::Vector{Int}, alltimes::Vector{Float64}, allorders::Vector{Tuple{Int64,Int64,Base.Int64}}, localrset::Vector{TempRouteSet}, dualinfo)

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

    #Backtrack through the levels (i.e. traverse route backwards) to find passengers and transferss
    for pl in ll:-1:1
        prev_label = labellookup[se.prevlist_l[pl]]
        pre_vertex = prev_label.n_l
        if pre_vertex <= n && pre_vertex > 0 
            if prev_label.BUS_station_l in Hub_Possiblematch_station[pre_vertex]
                ff = f_jht[pre_vertex, prev_label.BUS_station_l, Int(ceil(prev_label.T_l / unit))] # !!! Added to be int on 4-5
                push!(presetb2_r, ff)
            end
        end
        if pre_vertex > n && pre_vertex < 2 * n + 1 
            if prev_label.BUS_station_l in Possiblematch_station_d[pre_vertex - n]
                if !(pre_vertex - n in prev_label.preset1transfer_ll)
                    push!(preseta_r, pre_vertex - n)
                    g_r += oset[pre_vertex - n].g
                end
            end

            if prev_label.BUS_station_l in Hub_Possiblematch_station[pre_vertex - n]
                ff = f_jht[(pre_vertex - n), prev_label.BUS_station_l, Int(ceil(prev_label.T_l / unit))] # !!! Added to be int on 4-5
                push!(presetb1_r, ff)
                g_r += oset[pre_vertex - n].g # -0.1
            end
        end
    end
    g_r = g_r + ((-se.T_l + Start_time) / unit) * unit * cost

    #Calculate reduced cost
    routereducedcost = (g_r 
        + dualinfo.alpha1[start_r] - dualinfo.alpha1[end_r]
        - sum(dualinfo.beta[j] for j in preseta_r; init=0) - sum(dualinfo.beta[j] for j in 1:n if is_noempty_intersect(presetb1_r, fjset[j]); init=0) 
        - sum(dualinfo.gamma[j] for j in 1:n if is_noempty_intersect(presetb1_r, fjset[j]); init=0) + sum(dualinfo.gamma[j] for j in 1:n if is_noempty_intersect(presetb2_r, fjset[j]) ; init=0)
        - sum(dualinfo.delta[ff] for ff in presetb1_r; init=0) + sum(dualinfo.delta[ff] for ff in 1:f if is_noempty_intersect(presetb2_r, ffset[ff]); init=0) )
    
    #Check if route has negative reduced cost
    if routereducedcost > 0.0001
        push!(localrset, TempRouteSet(start_r, end_r, g_r, preseta_r, presetb1_r, presetb2_r, nodeset_r, customer_pudo_r, routereducedcost))
    end

    return localrset
end

#=
rr = length(rset)
info = rset[rr]
reduced_cost(z_r[rr])
routereducedcost = (info.g_r 
    + dualinfo.alpha1[info.start_r] - dualinfo.alpha1[info.end_r]
    - sum(dualinfo.beta[j] for j in info.preseta_r; init=0) - sum(dualinfo.beta[j] for j in 1:n if is_noempty_intersect(info.presetb1_r, fjset[j]); init=0) 
    - sum(dualinfo.gamma[j] for j in 1:n if is_noempty_intersect(info.presetb1_r, fjset[j]); init=0) + sum(dualinfo.gamma[j] for j in 1:n if is_noempty_intersect(info.presetb2_r, fjset[j]) ; init=0)
    - sum(dualinfo.delta[ff] for ff in info.presetb1_r; init=0) + sum(dualinfo.delta[ff] for ff in 1:f if is_noempty_intersect(info.presetb2_r, ffset[ff]); init=0) )
=#


#---------------------------------------------------------------------------------------#

"""
Function to set station status (forbidden, continue dropoff) before DP extension
"""
function Procedure_DP_set_station_loop(se::LabelSet2, lset::Vector{LabelSet2}, labellookup::Dict{Int,LabelSet2}, ll::Int)

    bus_ii = se.BUS_station_l
    # if the station has order pick-up still not dropoff, then add all possible order origins to BUS_station_forbidden
    if se.S_l >= 1
        for pl in ll:-1:1                                        #For all previous label-setting levels
            prev_label = labellookup[se.prevlist_l[pl]]
            pre_vertex = prev_label.n_l                                 #pre_vertex = order picked up or dropped off at this stop
            if pre_vertex <= n && pre_vertex > 0                      #If this is a real order (not a dummy order flag of some kind)
                if pre_vertex in se.preset1_ll                              #If we've picked up this order and not yet dropped it off
                    if pre_vertex in se.preset1transfer_ll                        #If it was a transfer pick up
                        this_station = prev_label.BUS_station_l                         #Find the station where this order was picked up
                        if this_station != bus_ii                                     #Add it to the forbidden list (unless it's this station)
                            push!(se.BUS_station_forbidden, this_station)
                        end
                    else                                                          #If it was an original pickup
                        for check_bus_jj in Possiblematch_station_o[pre_vertex]       #Add all the stations that order could've been picked up from to the forbidden list 
                            if check_bus_jj != bus_ii                                 # (unless we're at one of those now)
                                push!(se.BUS_station_forbidden, check_bus_jj)
                            end
                        end
                    end
                end
            end
        end
    end
    if bus_ii in se.BUS_station_continue_dropoff                 #If current station is the drop-off point for any passengers on the bus 
        check = 1
        for j in drelate_j_station[bus_ii]                       #For orders (jj) with bus_ii as a possible destination (i.e. bus_ii is in Possiblematch_station_d[jj] )
            if bus_ii in Possiblematch_station_d[j - n]          #j - n --> appears to be just an indexing thing for orders
                if j - n in se.preset1_ll                        #Order jj has been picked up but not dropped off
                    check = -1
                    break
                end
            end
        end
        if check == 1                                            #If there are no more orders that need to be dropped off at this station
            delete!(se.BUS_station_continue_dropoff, bus_ii)     #Then remove the station from the continue_dropoff list
        end
    end
end

#---------------------------------------------------------------------------------------#
#=Start_node=n_location_s[s_1] 
Start_time=time_s[s_1] * unit
index_start=s_1
ll=1=#

"""
Function to produce time-space arcs with given start node and time (Main Part of dynamic programming)

### input
* `lset` - label setting set
* `Start_node` - station id
* `Start_time` - time
* `index_start` - time-space node id
* `ll` - label setting recursion depth
"""

function Procedure_DP_for_generating_arcs_cg_loop(Start_node::Int, Start_time::Int, index_start::Int, localrset::Vector{TempRouteSet}, dualinfo) #lset::Vector{LabelSet2}, ll::Int, printstatements::Int,dualinfo)
    
    proc_begtime = now()
    #initialize the label set
    lset = Array{LabelSet2, 1}()
    labellookup = Dict{Int, LabelSet2}()
    push!(lset, get_Init_LabelSet2(Start_node, Start_time))
    labellookup[1] = get_Init_LabelSet2(Start_node, Start_time)

    while lset != []

        se = popfirst!(lset) # get currnet label status
        bus_ii = se.BUS_station_l # get current station
        ll = se.S_l

        # if not the initial status and no passengers here, then add the arc and end the recursio
        if se.Q_l == 0 && se.S_l > 0
            allstations, alltimes, allorders = getfullroute_loop(se, ll, labellookup)
            localrset = Procedure_DP_for_check_arc_cg_loop(se, labellookup, Start_time, index_start, ll, allstations, alltimes, allorders, localrset, dualinfo)
            return localrset
        end  
        
        # set station status before DP extension (i.e. update se.BUS_station_continue_dropoff and se.BUS_station_forbidden)
        Procedure_DP_set_station_loop(se, lset, labellookup, ll)
    
        # recursively extend the label for each bus station, be careful we need to start from bus_ii, the current station
        for bus_jj in union(bus_ii:TOTAL_Station, bus_ii - 1:-1:1)
            lset, labellookup, localrset = Procedure_DP_for_bus_jj_cg_loop(se, lset, labellookup, Start_node, ll, bus_jj, localrset)
        end

    end

    proc_chunk4_time = now()

    total_interval = proc_chunk4_time - proc_begtime
    #if (total_interval.value / 1000) >= 30
    #    println("Total time is $(total_interval.value / 1000)")
    #end

    return localrset
end


#---------------------------------------------------------------------------------------#

function getfullroute_loop(se, ll, labellookup)

    allstations, alltimes = Int[], Float64[]
    customerpickupdropoffs = Dict()
    allorders, neworders = [], Tuple{Int64,Int64,Int64}[]

    for level in ll:-1:1
        prev_label = labellookup[se.prevlist_l[level]]
        pushfirst!(allstations, prev_label.BUS_station_l)
        pushfirst!(alltimes, prev_label.T_l)
        push!(allorders, prev_label.n_l)
    end
    for j in allorders
        customerpickupdropoffs[j] = [-1,-1]
    end
    for level in ll:-1:1
        prev_label = labellookup[se.prevlist_l[level]]
        if prev_label.n_l > n
            customerpickupdropoffs[prev_label.n_l - n][2] = prev_label.BUS_station_l
        else
            customerpickupdropoffs[prev_label.n_l][1] = prev_label.BUS_station_l
        end
    end

    #Remove duplicates from route list
    newstations, newtimes = Int[], Float64[]
    push!(newstations, allstations[1])
    push!(newtimes, alltimes[1])
    for ind in 2:length(allstations)
        if allstations[ind] != allstations[ind-1]
            push!(newstations, allstations[ind])
            push!(newtimes, alltimes[ind])
        end
    end

    #Format the order pickup dropoffs
    for j in allorders
        if 0 < j < n
            push!(neworders, (j, customerpickupdropoffs[j][1], last(customerpickupdropoffs[j])))
        end
    end

    return newstations, newtimes, neworders

end

#---------------------------------------------------------------------------------------#

function solvesubproblem_loop(dualinfo)

    lset = Vector{LabelSet2}(undef, L)
    bestroute = nothingroute
    
    Time_longest_dp = -1
    begin_dp_total = now()
    end_dp_total = now()

    num_dp = 0
    #("r is: ", length(rset))
    enumstarttime = time()
    #println("Beginning the thing (s = $s)")
    for s_1 in 2:s
        if n_location_s[s_1] != TOTAL_Station + 1 && n_location_s[s_1] != 0 
            num_dp += 1
            begin_dp = now()

            localrset = Vector{TempRouteSet}()

            r = length(localrset)
            #println("r before procedure is: $r")
           
            stm=time()
            #Find all negative reduced cost routes
            localrset = Procedure_DP_for_generating_arcs_cg_loop(n_location_s[s_1], time_s[s_1] * unit, s_1, localrset, dualinfo)
            ttm = time()-stm
            #if ttm > 0.1
            #    println("s_1 = $s_1 --> ", ttm)
            #end
            r = length(localrset)
            #println("r after procedure is: $r for $s_1")

            #Check for new best route
            for rr in localrset
                if rr.reduced_cost > bestroute.reduced_cost + 1e-4
                    bestroute = rr
                end
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
    #println("Ending the thing")
    #println("Basic enumeration = ", time() - enumstarttime)
        
    #println("num_dp = $num_dp")
        
    end_dp_total = now()
    interval_dp_total = end_dp_total - begin_dp_total
    #println("*******************************************************************")
    #println("TotalTotalTotalTotal >> DP_total DONE! The Totmin is $(interval_dp_total.value / (1000 * 60)) or Totsec: $(interval_dp_total.value / (1000))")
    #println("*******************************************************************")
    #println("Parallelize >> DP_total DONE! The Totmin is $(Time_longest_dp / 60) or Totsec: $Time_longest_dp")

    return bestroute

end