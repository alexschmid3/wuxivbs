
function get_Init_LabelSet3(Start_node::Int, Start_time::Int)
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

    last_check = -1
    dist_since_last_check = 0.0

    return LabelSet3(n_l, T_l, S_l, Q_l, prel_l, Dwell_l, Transfer_l, BUS_station_l, preset1_ll, preset2_ll, preset1transfer_ll, BUS_station_continue_dropoff, BUS_station_start_pickup, BUS_station_forbidden, last_check, dist_since_last_check)

end

#---------------------------------------------------------------------------------------#

function check_New_LabelJ(se::LabelSet3, bus_jj::Int, j::Int, ln::Int)
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
    # ========================= Feasibility check 4: time deviation

    # ========================= Feasibility check 5: space deviation and station skips

    check_spacedeviation = -1
    stationsequence = referencelines[ln][1:length(referencelines[ln])-1]
    if se.last_check == -1
        lastcheckpoint = last(referencelines[ln][1:length(referencelines[ln])-1])
    else
        lastcheckpoint = se.last_check
    end
    for ss in 0:maxskipstations
        nextcheckpoint = stationsequence[mod(findfirst(x->x==lastcheckpoint,stationsequence)-1+ss, length(stationsequence)) + 1]
        if se.dist_since_last_check + Intersection_car_d_ij[bus_jj, nextcheckpoint] - Intersection_car_d_ij[lastcheckpoint, nextcheckpoint] <= maxdeviation_space*(ss+1)
            check_spacedeviation = 1
            break
        end
    end
    if check_spacedeviation == -1
        return false
    end

    return true
end

#---------------------------------------------------------------------------------------#

"""
Function to get new label to next bus station
"""
function get_New_LabelJ(se::LabelSet3, e::Int, bus_jj::Int, j::Int, ln::Int)
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
    last_check = bus_jj in referencelines[ln] ? bus_jj : se.last_check
    dist_since_last_check = bus_jj in referencelines[ln] ? 0 : se.dist_since_last_check + Intersection_car_d_ij[bus_ii,bus_jj]

    BUS_station_continue_dropoff = copy(se.BUS_station_continue_dropoff)
    BUS_station_start_pickup = copy(se.BUS_station_start_pickup)
    BUS_station_forbidden = Set{Int}()

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
        return LabelSet3(n_l, T_l, S_l, Q_l, prel_l, Dwell_l, Transfer_l, BUS_station_l, preset1_ll, preset2_ll, preset1transfer_ll, BUS_station_continue_dropoff, BUS_station_start_pickup, BUS_station_forbidden, last_check, dist_since_last_check)
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
        return LabelSet3(n_l, T_l, S_l, Q_l, prel_l, Dwell_l, Transfer_l, BUS_station_l, preset1_ll, preset2_ll, preset1transfer_ll, BUS_station_continue_dropoff, BUS_station_start_pickup, BUS_station_forbidden, last_check, dist_since_last_check)
    end

end

#---------------------------------------------------------------------------------------#

"""
Function to add a new time-space arc
"""
function Procedure_DP_for_add_arc(ln::Int, lset::Vector{LabelSet3}, Start_time::Int, index_start::Int, ll::Int, allstations::Vector{Int}, alltimes::Vector{Float64}, allorders::Vector{Tuple{Int64,Int64,Base.Int64}})
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
    push!(rset[ln], RouteSet(start_r, end_r, g_r, preseta_r, presetb1_r, presetb2_r, nodeset_r, customer_pudo_r))
end

#---------------------------------------------------------------------------------------#

"""
Function to set station status (forbidden, continue dropoff) before DP extension
"""
function Procedure_DP_set_station(lset::Vector{LabelSet3}, ll::Int)
    se = lset[ll]
    bus_ii = se.BUS_station_l
    # if the station has order pick-up still not dropoff, then add all possible order origins to BUS_station_forbidden
    if se.S_l >= 1
        for pl in ll:-1:1                                        #For all previous label-setting levels
            pre_vertex = lset[pl].n_l                                 #pre_vertex = order picked up or dropped off at this stop
            if pre_vertex <= n && pre_vertex > 0                      #If this is a real order (not a dummy order flag of some kind)
                if pre_vertex in se.preset1_ll                              #If we've picked up this order and not yet dropped it off
                    if pre_vertex in se.preset1transfer_ll                        #If it was a transfer pick up
                        this_station = lset[pl].BUS_station_l                         #Find the station where this order was picked up
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

"""
Function to extend label at the bus station
"""
function Procedure_DP_for_bus_jj(ln::Int, lset::Vector{LabelSet3}, Start_node::Int, Start_time::Int, index_start::Int, e::Int, ll::Int, bus_jj::Int, printstatements::Int)
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
                                sl = get_New_LabelJ(se, e, bus_jj, j, ln)            #New label 
                                if check_New_LabelJ(sl, bus_jj, j, ln)
                                    lset[ll] = sl
                                    Procedure_DP_for_generating_arcs(ln, lset, Start_node, Start_time, index_start, ll, printstatements)
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
                                sl = get_New_LabelJ(se, e, bus_jj, j, ln)
                                if check_New_LabelJ(sl, bus_jj, j, ln)
                                    lset[ll] = sl
                                    Procedure_DP_for_generating_arcs(ln, lset, Start_node, Start_time, index_start, ll, printstatements)
                                end
                                ll -= 1
                            end
                        end
                    end
                end
            end
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

function Procedure_DP_for_generating_arcs(ln::Int, lset::Vector{LabelSet3}, Start_node::Int, Start_time::Int, index_start::Int, ll::Int, printstatements::Int)
    
    proc_begtime = now()
    # if depth equals 1, need to initialize the label set
    if ll == 1
        lset[1] = get_Init_LabelSet3(Start_node, Start_time)
    end

    e = ll # record the depth
    se = lset[e] # get currnet label status
    bus_ii = se.BUS_station_l # get current station
    lastcheckpoint = se.last_check

    # if not the initial status and no passengers here, then add the arc and end the recursion
    if se.Q_l == 0 && se.S_l > 0
        allstations, alltimes, allorders = getfullroute(lset, ll)
        Procedure_DP_for_add_arc(ln, lset, Start_time, index_start, ll, allstations, alltimes, allorders)
        return
    end                                                                                             
 
    # set station status before DP extension (i.e. update se.BUS_station_continue_dropoff and se.BUS_station_forbidden)
    Procedure_DP_set_station(lset, ll)
    
    # recursively extend the label for each bus station, be careful we need to start from bus_ii, the current station
    for bus_jj in union(bus_ii, potentialnextstop[ln, lastcheckpoint, bus_ii]) #union(bus_ii:TOTAL_Station, bus_ii - 1:-1:1) 
        Procedure_DP_for_bus_jj(ln, lset, Start_node, Start_time, index_start, e, ll, bus_jj, printstatements)
    end

    proc_chunk4_time = now()

    total_interval = proc_chunk4_time - proc_begtime
    if (total_interval.value / 1000) >= 30
        println("Total time is $(total_interval.value / 1000)")
    end
end

#---------------------------------------------------------------------------------------#

function getfullroute(lset, ll)

    allstations, alltimes = Int[], Float64[]
    customerpickupdropoffs = Dict()
    allorders, neworders = [], Tuple{Int64,Int64,Int64}[]


    for level in ll:-1:1
        pushfirst!(allstations, lset[level].BUS_station_l)
        pushfirst!(alltimes, lset[level].T_l)
        push!(allorders, lset[level].n_l)
    end
    for j in allorders
        customerpickupdropoffs[j] = [-1,-1]
    end
    for level in ll:-1:1
        if lset[level].n_l > n
            customerpickupdropoffs[lset[level].n_l - n][2] = lset[level].BUS_station_l
        else
            customerpickupdropoffs[lset[level].n_l][1] = lset[level].BUS_station_l
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

"""
Function to get transfer order-time-hub nodes id of each order j 
"""
function get_transfer_pick_dropj(n::Int, f::Int)
    jset = [Set{Int}() for _ in 1:n]
    for ff in 1:f
        j = n_location_f[ff]
        if F_start_j[j] <= ff && ff <= F_end_j[j]
            push!(jset[j], ff)
        end
    end
    return jset
end

#---------------------------------------------------------------------------------------#

"""
Function to judge if the intersect of two sets is not empty 
"""
function is_noempty_intersect(S1::Set{Int}, S2::Set{Int})
    for ss in S1
        if ss in S2
            return true
        end
    end
    return false
end

#---------------------------------------------------------------------------------------#

"""
Function to get node set (with same order, same hub and time after node f) of each order-time-hub node f
"""
function get_transfer_pickf(f::Int)
    fset = [Set{Int}() for _ in 1:f]
    for ff in 1:f #iterate over possible trsnsfer nodes (each ff represents an order-time-hub with a feasible transfer)
        for ff_after in ff+1:f
            if n_location_f[ff_after] == n_location_f[ff] && hub_f[ff_after] == hub_f[ff] && time_f[ff_after] > time_f[ff]
                push!(fset[ff], ff_after)
            else
                break
            end
        end
    end
    return fset
end

#---------------------------------------------------------------------------------------#

"""
Function to get indexes of each constraint of the model  
"""
function get_indexes_Classic()

    A, B, C = Dict(), Dict(), Dict()
    R_minus, R_plus, R_minus_dummy, R_plus_dummy = Dict(), Dict(), Dict(), Dict()
    R_allpickups, R_transpickups, R_transdropoffs, R_one, R_two = Dict(), Dict(), Dict(), Dict(), Dict()
    
    ffset = get_transfer_pickf(f)
    fjset = get_transfer_pick_dropj(n, f)

    for ln in 1:numlines
        r = length(rset[ln])
        A[ln] = [ss for ss in 1:s if n_location_s[ss] != TOTAL_Station + 1 && n_location_s[ss] != 0]
        R_minus[ln] = [Vector{Int}() for _ = 1:s]
        R_plus[ln] = [Vector{Int}() for _ = 1:s]
        B[ln] = [ss for ss in 1:s if n_location_s[ss] == 0]
        C[ln] = [ss for ss in 1:s if n_location_s[ss] == TOTAL_Station + 1]
        R_minus_dummy[ln] = [Vector{Int}() for _ = 1:s]
        R_plus_dummy[ln] = [Vector{Int}() for _ = 1:s]
        R_allpickups[ln] = [Vector{Int}() for _ in 1:n]
        R_transpickups[ln] = [Vector{Int}() for _ in 1:n]
        R_transdropoffs[ln] = [Vector{Int}() for _ in 1:n]
        R_one[ln] = [Vector{Int}() for _ in 1:f]
        R_two[ln] = [Vector{Int}() for _ in 1:f]

        for rr in 1:timespace_route1[ln]
            push!(R_minus[ln][rset[ln][rr].start_r], rr)
            push!(R_plus[ln][rset[ln][rr].end_r], rr)
            @inbounds for j in rset[ln][rr].preseta_r
                push!(R_allpickups[ln][j], rr)
            end
            @inbounds for ff in rset[ln][rr].presetb1_r
                push!(R_one[ln][ff], rr)
            end
        end

        for rr in 1:timespace_route1[ln]
            @inbounds for j in 1:n
                if is_noempty_intersect(rset[ln][rr].presetb1_r, fjset[j])
                    push!(R_transpickups[ln][j], rr)
                    push!(R_allpickups[ln][j], rr)
                end
                if is_noempty_intersect(rset[ln][rr].presetb2_r, fjset[j])
                    push!(R_transdropoffs[ln][j], rr)
                end
            end
            @inbounds for ff in 1:f
                if is_noempty_intersect(rset[ln][rr].presetb2_r, ffset[ff])
                    push!(R_two[ln][ff], rr)
                end
            end
        end

        @inbounds for rr in (timespace_route1[ln] + 1):r-1
            push!(R_minus[ln][rset[ln][rr].start_r], rr)
            push!(R_plus[ln][rset[ln][rr].end_r], rr)
            push!(R_minus_dummy[ln][rset[ln][rr].start_r], rr)
            push!(R_plus_dummy[ln][rset[ln][rr].end_r], rr)
        end
    end

    println("====================================")
    return A, R_minus, R_plus, B, C, R_minus_dummy, R_plus_dummy, R_allpickups, R_transpickups, R_transdropoffs, R_one, R_two
end


#---------------------------------------------------------------------------------------#

"""
Function to get indexes of each constraint of the model  
"""
function get_indexes_Classic_cg()
    r = length(rset)
    A = [ss for ss in 1:s if n_location_s[ss] != TOTAL_Station + 1 && n_location_s[ss] != 0]
    R_minus = [Vector{Int}() for _ = 1:s]
    R_plus = [Vector{Int}() for _ = 1:s]
    B = [ss for ss in 1:s if n_location_s[ss] == 0]
    C = [ss for ss in 1:s if n_location_s[ss] == TOTAL_Station + 1]
    R_minus_dummy = [Vector{Int}() for _ = 1:s]
    R_plus_dummy = [Vector{Int}() for _ = 1:s]
    R_allpickups = [Vector{Int}() for _ in 1:n]
    R_transpickups = [Vector{Int}() for _ in 1:n]
    R_transdropoffs = [Vector{Int}() for _ in 1:n]
    R_one = [Vector{Int}() for _ in 1:f]
    R_two = [Vector{Int}() for _ in 1:f]

    ffset = get_transfer_pickf(f)
    fjset = get_transfer_pick_dropj(n, f)

    for rr in timespace_route2-timespace_route1+1:timespace_route2
        push!(R_minus[rset[rr].start_r], rr)
        push!(R_plus[rset[rr].end_r], rr)
        @inbounds for j in rset[rr].preseta_r
            push!(R_allpickups[j], rr)
        end
        @inbounds for ff in rset[rr].presetb1_r
            push!(R_one[ff], rr)
        end
    end

    for rr in timespace_route2-timespace_route1+1:timespace_route2
        @inbounds for j in 1:n
            if is_noempty_intersect(rset[rr].presetb1_r, fjset[j])
                push!(R_transpickups[j], rr)
                push!(R_allpickups[j], rr)
            end
            if is_noempty_intersect(rset[rr].presetb2_r, fjset[j])
                push!(R_transdropoffs[j], rr)
            end
        end
        @inbounds for ff in 1:f
            if is_noempty_intersect(rset[rr].presetb2_r, ffset[ff])
                push!(R_two[ff], rr)
            end
        end
    end

    @inbounds for rr in 1:timespace_route2-timespace_route1
        push!(R_minus[rset[rr].start_r], rr)
        push!(R_plus[rset[rr].end_r], rr)
        push!(R_minus_dummy[rset[rr].start_r], rr)
        push!(R_plus_dummy[rset[rr].end_r], rr)
    end

    println("=====================-------------------===============")
    return A, R_minus, R_plus, B, C, R_minus_dummy, R_plus_dummy, R_allpickups, R_transpickups, R_transdropoffs, R_one, R_two
end

#---------------------------------------------------------------------------------------#

"""
Function to get the model solve result 
"""
function get_optimal_plan(model)
    z_r_opt = value.(model[:z_r])
    opt_indexes = Dict()
    for ln in 1:numlines
        opt_indexes[ln] = []
        for rr in 1:length(rset[ln])
            if z_r_opt[ln,rr] > 0.99
                push!(opt_indexes[ln], rr)
            end
        end
    end
   
    for ln in 1:numlines, rr in opt_indexes[ln]
        for j in 1:n
            #if j in rset[ln][rr].preseta_r
            #    println("rr = ", rr, " includes customer ", j)
            #end
            for ff in 1:f
                if n_location_f[ff] == j
                    if ff in rset[ln][rr].presetb1_r
                        println("rr = ", rr, " includes transfer pick customer ", j)
                    end
                    if ff in rset[ln][rr].presetb2_r
                        println("rr = ", rr, " includes transfer drop customer ", j)
                    end
                end
            end
        end
    end
    return opt_indexes
end

#---------------------------------------------------------------------------------------#

"""
Function to get gurobi termination status 
"""
function termination(model)
    if termination_status(model) == MOI.OPTIMAL
        optimal_objective = objective_value(model)
        gap = relative_gap(model)
        lower_bound = objective_bound(model)
    elseif termination_status(model) == MOI.INFEASIBLE_OR_UNBOUNDED
        optimal_objective = Inf
        gap = Inf
        lower_bound = Inf
    elseif termination_status(model) == MOI.TIME_LIMIT 
        lower_bound = objective_bound(model)
        if has_values(model)
            optimal_objective = objective_value(model)
            gap = relative_gap(model)
        else
            optimal_objective = Inf
            gap = Inf
        end
    else
        println("Unexpected model termination status: ", string(termination_status(model)))
        if has_values(model)
            lower_bound = objective_bound(model)
            optimal_objective = objective_value(model)
            gap = relative_gap(model)
        else
            optimal_objective = Inf
            gap = Inf
            lower_bound = -Inf
        end
    end
    return optimal_objective, gap, lower_bound
end

#---------------------------------------------------------------------------------------#

"""
Function to solve the model  
"""
function solve_network_model_reflines(W::Int)
    println("**************************** Begin IO *****************************")
    # Start timer
    begin_gurobi = now()
    
    # Initialize the model with the Gurobi optimizer
    model = Model(Gurobi.Optimizer)
    set_optimizer_attribute(model, "TimeLimit", 7200)  # 2 hours time limit
    set_optimizer_attribute(model, "MIPGap", 0.00001)  # Relative MIP optimality gap
    set_optimizer_attribute(model, "OutputFlag", gurobioutput_flag)
    
    # Variables
    # Define z_r: A boolean variable array
    @variable(model, z_r[ln in 1:numlines, 1:length(rset[ln])], Bin);

    #@variable(model, 0 <= Λ <= W) 

    # Constraints
    @views N_flow, R_minus, R_plus, N_dummyorig, N_dummydest, R_minus_dummy, R_plus_dummy, R_allpickups, R_transpickups, R_transdropoffs, R_one, R_two = get_indexes_Classic();
    @constraint(model, flowbalance1[ln in 1:numlines, ss=N_flow[ln]], sum(z_r[ln,rr] for rr in R_minus[ln][ss]) == sum(z_r[ln,rr] for rr in R_plus[ln][ss]));
    @constraint(model, flowbalance2[ln in 1:numlines, ss=N_dummyorig[ln]], sum(z_r[ln,rr] for rr in R_minus_dummy[ln][ss]) - sum(z_r[ln,rr] for rr in R_plus_dummy[ln][ss]) == 1);
    @constraint(model, flowbalance3[ln in 1:numlines, ss=N_dummydest[ln]], sum(z_r[ln,rr] for rr in R_minus_dummy[ln][ss]) - sum(z_r[ln,rr] for rr in R_plus_dummy[ln][ss]) == -1);
    @constraint(model, ordersserved[j=1:n], sum(sum(z_r[ln,rr] for rr in R_allpickups[ln][j]) for ln in 1:numlines) <= 1);
    @constraint(model, transferconsistency[j=1:n], sum(sum(z_r[ln,rr] for rr in R_transpickups[ln][j]) for ln in 1:numlines) == sum(sum(z_r[ln,rr] for rr in R_transdropoffs[ln][j]) for ln in 1:numlines));
    @constraint(model, dropoffafterpickup[ff=1:f], sum(sum(z_r[ln,rr] for rr in R_one[ln][ff]) for ln in 1:numlines) <= sum(sum(z_r[ln,rr] for rr in R_two[ln][ff]) for ln in 1:numlines));

    # objective
    @objective(model, Max, sum(sum(z_r[ln,rr] * rset[ln][rr].g_r for rr in 1:length(rset[ln])) for ln in 1:numlines));

    # Solve the model
    optimize!(model)

    optimal_objective, gap, upper_bound = termination(model) 
    r_opt = get_optimal_plan(model)
    
    # Convert milliseconds to seconds, minutes, and hours
    end_gurobi = now()
    interval_gurobi = end_gurobi - begin_gurobi
    seconds = interval_gurobi ÷ 1000  # Integer division to get whole seconds
    minutes = seconds ÷ 60  # Convert seconds to minutes
    remaining_seconds = seconds % 60  # Remaining seconds after converting to minutes
    hours = minutes ÷ 60  # Convert minutes to hours
    remaining_minutes = minutes % 60  # Remaining minutes after converting to hours

    println("*******************************************************************")
    println("Gurobi DONE! The Total Time is $(hours.value) hours, $(remaining_minutes.value) minutes, $(remaining_seconds.value) seconds")
    println("*******************************************************************")
    
    customers_served = sum(sum(value(z_r[ln,rr]) * length(rset[ln][rr].preseta_r) for rr in 1:length(rset[ln])) for ln in 1:numlines) + sum(sum(value(z_r[ln,rr]) * length(rset[ln][rr].presetb1_r) for rr in 1:length(rset[ln])) for ln in 1:numlines)
    customers_transferred = sum(sum(value(z_r[ln,rr]) * length(rset[ln][rr].presetb1_r) for rr in 1:length(rset[ln])) for ln in 1:numlines)
    
    return optimal_objective, gap, upper_bound, r_opt, solve_time(model), customers_served, customers_transferred
end

#---------------------------------------------------------------------------------------#
#=
"""
Function to solve the model  
"""
function solve_network_model_cg(W::Int)
    println("**************************** Begin IO *****************************")
    # Start timer
    begin_gurobi = now()
    
    # Initialize the model with the Gurobi optimizer
    model = Model(Gurobi.Optimizer)
    set_optimizer_attribute(model, "TimeLimit", 7200)  # 2 hours time limit
    set_optimizer_attribute(model, "MIPGap", 0.00001)  # Relative MIP optimality gap
    
    r = length(rset)
    # Variables
    # Define z_r: A boolean variable array
    @variable(model, z_r[1:r], Bin)
    # Define vehicle_no: An integer variable constrained between 0 and W
    @variable(model, 0 <= Λ <= W) 

    # Constraints
    @views N_flow, R_minus, R_plus, N_dummyorig, N_dummydest, R_minus_dummy, R_plus_dummy, R_allpickups, R_transpickups, R_transdropoffs, R_one, R_two = get_indexes_Classic_cg()
    @constraint(model, flowbalance1[ss=N_flow], sum(z_r[rr] for rr in R_minus[ss]) == sum(z_r[rr] for rr in R_plus[ss]))
    @constraint(model, flowbalance2[ss=N_dummyorig], sum(z_r[rr] for rr in R_minus_dummy[ss]) - sum(z_r[rr] for rr in R_plus_dummy[ss]) == Λ)
    @constraint(model, flowbalance3[ss=N_dummydest], sum(z_r[rr] for rr in R_minus_dummy[ss]) - sum(z_r[rr] for rr in R_plus_dummy[ss]) == -Λ)
    @constraint(model, ordersserved[j=1:n], sum(z_r[rr] for rr in R_allpickups[j]) <= 1)
    @constraint(model, transferconsistency[j=1:n], sum(z_r[rr] for rr in R_transpickups[j]) == sum(z_r[rr] for rr in R_transdropoffs[j]))
    @constraint(model, dropoffafterpickup[ff=1:f], sum(z_r[rr] for rr in R_one[ff]) <= sum(z_r[rr] for rr in R_two[ff]))

    # objective
    @objective(model, Max, sum(z_r[rr] * rset[rr].g_r for rr in 1:r) - 0.001 * Λ)

    # Solve the model
    optimize!(model)

    optimal_objective, gap, upper_bound = termination(model) 
    r_opt = get_optimal_plan(model)
    
    # Convert milliseconds to seconds, minutes, and hours
    end_gurobi = now()
    interval_gurobi = end_gurobi - begin_gurobi
    seconds = interval_gurobi ÷ 1000  # Integer division to get whole seconds
    minutes = seconds ÷ 60  # Convert seconds to minutes
    remaining_seconds = seconds % 60  # Remaining seconds after converting to minutes
    hours = minutes ÷ 60  # Convert minutes to hours
    remaining_minutes = minutes % 60  # Remaining minutes after converting to hours

    println("*******************************************************************")
    println("Gurobi DONE! The Total Time is $(hours.value):$(remaining_minutes.value):$(remaining_seconds.value)")
    println("*******************************************************************")
    
    customers_served = sum(value(z_r[rr]) * length(rset[rr].preseta_r) for rr in 1:r) + sum(value(z_r[rr]) * length(rset[rr].presetb1_r) for rr in 1:r)
    customers_transferred = sum(value(z_r[rr]) * length(rset[rr].presetb1_r) for rr in 1:r)
    
    return sum(value(z_r[rr]) * rset[rr].g_r for rr in 1:r), gap, upper_bound, r_opt, solve_time(model), value(Λ), customers_served, customers_transferred
end
=#