

include("initializenetwork.jl")
include("solvesubproblem_recursion.jl")
include("solvesubproblem_initialenum.jl")
include("solvesubproblem_loop.jl")

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
    for ff in 1:f
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

function initializermp()

    r = length(rset)
    N_flow = [ss for ss in 1:s if n_location_s[ss] != TOTAL_Station + 1 && n_location_s[ss] != 0]
    R_minus = [Vector{Int}() for _ = 1:s]
    R_plus = [Vector{Int}() for _ = 1:s]
    N_dummyorig = [ss for ss in 1:s if n_location_s[ss] == 0]
    N_dummydest = [ss for ss in 1:s if n_location_s[ss] == TOTAL_Station + 1]
    R_minus_dummy = [Vector{Int}() for _ = 1:s]
    R_plus_dummy = [Vector{Int}() for _ = 1:s]
    R_allpickups = [Vector{Int}() for _ in 1:n]
    R_transpickups = [Vector{Int}() for _ in 1:n]
    R_transdropoffs = [Vector{Int}() for _ in 1:n]
    R_one = [Vector{Int}() for _ in 1:f]
    R_two = [Vector{Int}() for _ in 1:f]

    ffset = get_transfer_pickf(f)
    fjset = get_transfer_pick_dropj(n, f)

    for rr in 1:timespace_route1
        push!(R_minus[rset[rr].start_r], rr)
        push!(R_plus[rset[rr].end_r], rr)
        @inbounds for j in rset[rr].preseta_r
            push!(R_allpickups[j], rr)
        end
        @inbounds for ff in rset[rr].presetb1_r
            push!(R_one[ff], rr)
        end
    end

    for rr in 1:timespace_route1
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

    #=
    @inbounds for rr in (timespace_route1 + 1):r
        push!(R_minus[rset[rr].start_r], rr)
        push!(R_plus[rset[rr].end_r], rr)
        push!(R_minus_dummy[rset[rr].start_r], rr)
        push!(R_plus_dummy[rset[rr].end_r], rr)
    end
    =#

    return N_flow, R_minus, R_plus, N_dummyorig, N_dummydest, R_minus_dummy, R_plus_dummy, R_allpickups, R_transpickups, R_transdropoffs, R_one, R_two

end

#-------------------------------------------------------------------#

function getdualinfo(rmpconstraints)

	alpha1 = dual.(rmpconstraints.con_flowbalance1)
	alpha2 = dual.(rmpconstraints.con_flowbalance2)
	alpha3 = dual.(rmpconstraints.con_flowbalance3)
	beta = dual.(rmpconstraints.con_ordersserved)
	gamma = dual.(rmpconstraints.con_transferconsistency)
	delta = dual.(rmpconstraints.con_dropoffafterpickup)
    #gamma = Array(dual.(rmpconstraints.con_transferconsistency)) --> transform to array if more efficient

    return alpha1, alpha2, alpha3, beta, gamma, delta

end

#-------------------------------------------------------------------#

function columngeneration()

    @views N_flow, R_minus, R_plus, N_dummyorig, N_dummydest, R_minus_dummy, R_plus_dummy, R_allpickups, R_transpickups, R_transdropoffs, R_one, R_two = initializermp()
    
    #----------------------------- MASTER PROBLEM -----------------------------#

    # Initialize the model with the Gurobi optimizer
    rmp = Model(Gurobi.Optimizer)
    set_optimizer_attribute(rmp, "TimeLimit", 7200)  # 2 hours time limit
    set_optimizer_attribute(rmp, "MIPGap", 0.0001)  # Relative MIP optimality gap
    set_optimizer_attribute(rmp, "OutputFlag", 0)
    
    r = length(rset)
    # Variables
    # Define z_r: A boolean variable array
    z_r = Dict()
	for rr in 1:r
	    global z_r[rr] = @variable(rmp, lower_bound = 0) #upper bound implied by ordersserved constraint                      
	    set_name(z_r[rr], string("z_r[",rr,"]")) 
	end
    # Define vehicle_no: An integer variable constrained between 0 and W
    @variable(rmp, 0 <= Λ <= W) 

    # Constraints
    @constraint(rmp, flowbalance1[ss=N_flow], sum(z_r[rr] for rr in R_minus[ss]) == sum(z_r[rr] for rr in R_plus[ss]))
    @constraint(rmp, flowbalance2[ss=N_dummyorig], sum(z_r[rr] for rr in R_minus_dummy[ss]) - sum(z_r[rr] for rr in R_plus_dummy[ss]) == Λ)
    @constraint(rmp, flowbalance3[ss=N_dummydest], sum(z_r[rr] for rr in R_minus_dummy[ss]) - sum(z_r[rr] for rr in R_plus_dummy[ss]) == -Λ)
    @constraint(rmp, ordersserved[j=1:n], sum(z_r[rr] for rr in R_allpickups[j]) <= 1)
    @constraint(rmp, transferconsistency[j=1:n], sum(z_r[rr] for rr in R_transpickups[j]) == sum(z_r[rr] for rr in R_transdropoffs[j]))
    @constraint(rmp, dropoffafterpickup[ff=1:f], sum(z_r[rr] for rr in R_one[ff]) <= sum(z_r[rr] for rr in R_two[ff]))

    # objective
    @objective(rmp, Max, sum(z_r[rr] * rset[rr].g_r for rr in 1:r))

    #Group constraints
    rmpconstraints = (con_flowbalance1 = flowbalance1,
		con_flowbalance2 = flowbalance2,
		con_flowbalance3 = flowbalance3,
		con_ordersserved = ordersserved,
		con_transferconsistency = transferconsistency,
		con_dropoffafterpickup = dropoffafterpickup
		)

    #------------------------------- MAIN LOOP --------------------------------#

    cg_iter = 1

    fullalgstarttime = time()
    mptime, sptime = 0, 0

    localrset = enumerateroutes()

    println("********************* Begin column generation *********************")

    while cg_iter <= 1000

		#-------------SOLVE RMP-------------#

        status = optimize!(rmp)
        rmpobj = objective_value(rmp)
        mptime += solve_time(rmp)
        
        #------------SUBPROBLEMS------------#

        #Get dual information
        alpha1, alpha2, alpha3, beta, gamma, delta = getdualinfo(rmpconstraints)
        dualinfo = (alpha1=alpha1, alpha2=alpha2, alpha3=alpha3, beta=beta, gamma=gamma, delta=delta)
        
        #Solve subproblem - find new route(s)
        sptimestart = time()
        newroute = solvesubproblem_initialenum(dualinfo, localrset)  
        sptime += time() - sptimestart

		#---------ADD NEW VARIABLES---------#

        if newroute != nothingroute

            #Add to sets
            push!(rset, RouteSet(newroute.start_r,newroute.end_r,newroute.g_r,newroute.preseta_r,newroute.presetb1_r,newroute.presetb2_r,newroute.nodepath_r,newroute.customer_pudo_r))
            rr = length(rset)
            global timespace_route1 += 1
            global timespace_route2 += 1
            #showsingleroute(string("viz/cg/cgiter", cg_iter, "_route",rr, ".png"), rr, 1800, 1800)

            #Add variable
            global z_r[rr] = @variable(rmp, lower_bound = 0)
			set_name(z_r[rr], string("z_r[",rr,"]")) 

			#Add to objective
			set_objective_coefficient(rmp, z_r[rr], newroute.g_r) 

            #Add to constraints
            set_normalized_coefficient(rmpconstraints.con_flowbalance1[newroute.start_r], z_r[rr], 1.0)
            set_normalized_coefficient(rmpconstraints.con_flowbalance1[newroute.end_r], z_r[rr], -1.0)
            for j in newroute.preseta_r
                set_normalized_coefficient(rmpconstraints.con_ordersserved[j], z_r[rr], 1.0)
            end
            for j in 1:n
                if is_noempty_intersect(newroute.presetb1_r, fjset[j])
                    set_normalized_coefficient(rmpconstraints.con_ordersserved[j], z_r[rr], 1.0)
                end
            end
            for j in 1:n
                if is_noempty_intersect(newroute.presetb1_r, fjset[j])
                    set_normalized_coefficient(rmpconstraints.con_transferconsistency[j], z_r[rr], 1.0)
                end
            end
            for j in 1:n
                if is_noempty_intersect(newroute.presetb2_r, fjset[j])
                    set_normalized_coefficient(rmpconstraints.con_transferconsistency[j], z_r[rr], -1.0)
                end
            end
            for ff in newroute.presetb1_r
                set_normalized_coefficient(rmpconstraints.con_dropoffafterpickup[ff], z_r[rr], 1.0)
            end
            for ff in 1:f
                if is_noempty_intersect(rset[rr].presetb2_r, ffset[ff])
                    set_normalized_coefficient(rmpconstraints.con_dropoffafterpickup[ff], z_r[rr], -1.0)
                end
            end

        end

		#------------TERMINATION------------#

        println("CG iteration $cg_iter r.c. = ", newroute.reduced_cost)
        if (newroute.reduced_cost <= 0.0001) 
			println("NO NEGATIVE REDUCED COSTS FOUND!")	
			break
		end

		#--------------ITERATE--------------#

		cg_iter += 1
	
	end

    fullalgtime = time() - fullalgstarttime

    optimize!(rmp)
    println("LO optimal objective = ", objective_value(rmp))

    println("********************** End column generation **********************")

    return objective_value(rmp), fullalgtime, cg_iter, mptime, sptime

end