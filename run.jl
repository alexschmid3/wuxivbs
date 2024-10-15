
using DataFrames, Dates, JuMP, Gurobi, CSV

# ---------------------------------------------------------------------------------

include("scripts/createstructs.jl")
include("scripts/Input_Overall.jl")
include("scripts/Create_time_space_nodes_and_arcs.jl")
include("scripts/solve_network_model.jl")

#Colum generation
include("scripts/columngeneration/columngeneration.jl")

#Visualization and reporting
include("scripts/metricsandvisuals/routeviz.jl")
include("scripts/metricsandvisuals/printouts.jl")

# ---------------------------------------------------------------------------------

distcarfilename = "data/dist-car-small.csv"                     #Driving distances
distpedestrianfilename = "data/dist-pedestrian-small.csv"       #Walking distances
nodesfilename = "data/nodes-small.csv"                          #Bus stop coordinates

# ---------------------------------------------------------------------------------

experiment_id = 2
paramsfilename = "data/experimentlist.csv"                      #List of instance parameters
expparms = CSV.read(paramsfilename, DataFrame)
customerfilename = string("data/customers/", expparms[experiment_id, 9])  
    #"data/customers/order-1218-hs-small.csv"  --> real data from Wuxi
    #"data/customers/order-syn-nK.csv"         --> synthetic data with K orders per 10 min, genearted by scripts/syntehticdata/generatesyntheticdata.jl

#---------------------------------------#
# File parameters
#---------------------------------------#
# vcc_type - "distance": max walking distance, "candidates": can walk to the nearest k stations
# vcc_candidates - number of VBS options for each customer, 1 <= vcc_candidates <= 5   (vcc_candidates = 1 means no VCC)
# vcc_maxdist - max walking distance, vcc_maxdist >= 0 (vcc_maxdist = 0 means no VCC)
# transfer_flag	- 0: no tranfers, 1: transfers allowed
# transfer_maxdev - max deviation from shortest path distance to drop off a transfer, transfer_maxdev >= 1.0 (e.g. transfer_maxdev=1.05 means only consider transfer hubs for each customer where origin->hub->destination distance is within 5% of the shortest path distance from origin->destiantion)
# W - max vehicles
# Q - vehicle capacity
# customerfilename	- file containing list of customer requests
# method - solution method, either "ip" or "cg" for now
# referencelines_flag - 0: no reference lines, 1: enforce reference lines
# maxskipstations - skip up to K checkpoints
# maxdeviation_space - max spatial deviation from reference line between checkpoints
# maxdeviation_time - [PLACEHOLDER - not functioning] max time deviation from reference line between checkpoints

const TOTAL_Station = 65                                        # all the bus station 
Q = expparms[experiment_id, 8]                                  
W = expparms[experiment_id, 7]                                 
const T_0 = 3600 * 3                                            # entire time horizon~
addsecond = 600                                                 # length of planning horizon (seconds)
previous_resolve_start = Int(round(3600 * 1.5, digits=0))       
resolve_start = Int(round(3600 * 1.5, digits=0) + addsecond)    
const Dwell_time = 60                                           
manualhubs = [51, 7, 3, 2, 1, 20, 63, 31, 38, 16]               # list of VBS that allow transfers ; old transfer hubs: [51, 7, 44, 41, 35, 25, 23, 63, 38]

F = 20000 # transfer
const sv = 15.0 * 1000.0 / 3600.0                               # vehicle speed /s
const cost = 3.2 / 3600.0                                       # cost /s (this is in dollars)
const discre_no = 5

const unit = 60                                                 # time-space network: time discretization
const S = 100000
const R = 3000000
const L = 1000

const DP_CPU_time::Int = 0
const CPLEX_CPU_time::Int = 0

#Solution method
method = expparms[experiment_id, 10]                            

#Toggle switches for VCC and transfers
vcc_type = expparms[experiment_id, 2]                           # "distance" - max walking distance, "candidates" - can walk to the nearest k stations 
vcc_candidates = expparms[experiment_id, 3]                     # number of VBS options for each customer, 1 <= vcc_candidates <= 5   (vcc_candidates = 1 means no VCC)
vcc_maxdist = expparms[experiment_id, 4]                        # max walking distance, vcc_maxdist >= 0 (vcc_maxdist = 0 means no VCC)
transfer_flag = expparms[experiment_id, 5]                      # 0 - no tranfers, 1 - transfers allowed
transfer_maxdev = expparms[experiment_id, 6]                    # max deviation from shortest path distance to drop off a transfer, transfer_maxdev >= 1.0 (e.g. transfer_maxdev=1.05 means only consider transfer hubs for each customer where origin->hub->destination distance is within 5% of the shortest path distance from origin->destiantion)
transfer_maxhubs = TOTAL_Station                                # max transfer hub candidates for each customer ; setting transfer_maxhubs = TOTAL_Station means no restriction

#Parameters for reference lines with deviation
referencelines_flag = expparms[experiment_id, 11]               # 0 - no reference lines, 1 - enforce reference lines
maxdeviation_time = expparms[experiment_id, 14]                 # [PLACEHOLDER - not yet functioning] max time deviation 
maxdeviation_space = expparms[experiment_id, 13]                # max spatial deviation from reference line between checkpoints
maxskipstations = expparms[experiment_id, 12]                   # skip up to K checkpoints
numlines = W                                                    # number of reference lines (and thus vehicles in our short term horizon problem)

#Reporting
viz_flag = 0
gurobioutput_flag = 0
outputfoldername = "outputs"
outputfilename = string(outputfoldername,"/output_exp", experiment_id,".csv")
if !(isdir(outputfoldername))
	mkdir(outputfoldername)
end
if !(isdir("viz"))
	mkdir("viz")
end

# ---------------------------------------------------------------------------------

if referencelines_flag == 1
    include("scripts/deviatedfixedlines/loadlines.jl")
    include("scripts/deviatedfixedlines/solve_network_model_reflines.jl")
    include("scripts/deviatedfixedlines/Create_time_space_nodes_and_arcs_reflines.jl")
    include("scripts/deviatedfixedlines/Input_Overall_reflines.jl")
    include("scripts/deviatedfixedlines/columngeneration_reflines.jl")
end

# ---------------------------------------------------------------------------------

oset = Vector{Order}()

#Information about time-space nodes
s = 0
n_location_s = zeros(Int, S) # space
time_s = zeros(Int, S) # time
s_st = zeros(Int, TOTAL_Station + 1, 10000) #space time index
nodelookup = Dict()

#Initialize instance inputs
n, Bus_station_travel_time, station_coordinates, Possiblematch_station_o, Possiblematch_station_d, Hub_Possiblematch_station, shortest_arrive_transfer_duration_jstation, shortest_arrive_destination_duration_jstation, shortest_direct_arrive_destination_duration_jstation, prelate_j_station, drelate_j_station, match_no, Intersection_car_d_ij, HUB_station, coverage, potentialnextstop = Input_Overall(transfer_flag, vcc_candidates)

#Visualize
if viz_flag == 1 
    routeviz("viz/hubs.png", 1800, 1800, [], [], [], [])
end
if referencelines_flag == 1 & viz_flag == 1
    visualizecoverage("viz/coverage.png",2000,2000)
end

# ---------------------------------------------------------------------------------

#Initialize sets for transfer data
f = 0 # transfer nodes
n_location_f = zeros(Int, F + 1) # transfer
time_f = zeros(Int, F + 1) # transfer
hub_f = zeros(Int, F + 1) # transfer

F_start_j = zeros(Int, n) # transfer
F_end_j = zeros(Int, n) # transfer
f_jht = zeros(Int, n, TOTAL_Station + 1, 10000) # transfer

if referencelines_flag == 0
    rset = Vector{RouteSet}()
elseif referencelines_flag == 1
    rset = Dict{Int64,Vector{RouteSet}}()
    for ln in 1:numlines
        rset[ln] =  Vector{RouteSet}()
    end
end

if referencelines_flag == 1
    timespace_route1, timespace_route2, timespace_route3 = Dict(), Dict(), Dict()
elseif referencelines_flag == 0
    timespace_route1, timespace_route2, timespace_route3 = 0, 0, 0
end
nothingroute = TempRouteSet(0,0,0.0,Set{Int}(), Set{Int}(), Set{Int}(), Array{Int}(undef), Array{Tuple{Int64, Int64, Int64}}(undef), 0.0)

# ---------------------------------------------------------------------------------

#Solve
if method == "ip"

    if referencelines_flag == 1
    
        #Create the time-space network
        sptime = Create_time_space_nodes_and_arcs_reflines(W) 
        
        #Solve model with reference lines
        ip_obj, gap, ub, r_opt, ip_time, customers_served, customers_transferred = solve_network_model_reflines(W)
    
    elseif referencelines_flag == 0

        #Create the time-space network
        sptime = Create_time_space_nodes_and_arcs(W) 

        #Solve model (no reference lines)
        ip_obj, gap, ub, r_opt, ip_time, vehicles_used, customers_served, customers_transferred = solve_network_model(W)
    
    end
    
    #For reporting purposes
    cg_iter, cg_time, mptime, cg_obj = 0, 0, 0, 0
    totaltime = ip_time + sptime

    printsolutioninfo(r_opt)

elseif method == "cg"

    if referencelines_flag == 1

        throw(DomainError(method, "No method = 'cg' if referencelines_flag = 1"))

    elseif referencelines_flag == 0

        #Initialize time-space network
        initializenetwork(W)  
        ffset = get_transfer_pickf(f)
        fjset = get_transfer_pick_dropj(n, f)

        #Solve LO via CG
        cg_obj, cg_time, cg_iter, mptime, sptime = columngeneration()

        #Solve IO with CG routes
        ip_obj, gap, ub, r_opt, ip_time, vehicles_used, customers_served, customers_transferred = solve_network_model_cg(W)
        totaltime = cg_time + ip_time  

    end

    printsolutioninfo(r_opt)

end

#Visualization and detour calculation
detour = processropt(r_opt, 1800, 1800)

#Tally the total number of routes
if referencelines_flag == 1
    totalroutes = sum(length(rset[ln]) for ln in 1:numlines)
elseif referencelines_flag == 0
    totalroutes = length(rset)
end
        
#Save results to CSV
df = DataFrame(experiment_id = [experiment_id],
                vcc_type = [vcc_type],
                vcc_candidates = [vcc_candidates],
                vcc_distance = [vcc_maxdist],
                transfer_flag = [transfer_flag],
                transfer_maxdev = [transfer_maxdev],
                maxtransferhubs = [transfer_maxhubs],
                referencelines_flag = [referencelines_flag], 
                numlines = [numlines], 
                maxskipstations = [maxskipstations], 
                maxdeviation_space = [maxdeviation_space], 
                maxdeviation_time = [maxdeviation_time], 
                n = [n],
                W = [W],
                Q = [Q],
                method = [method],
                numroutes = [totalroutes],
                LO_objective = [cg_obj],
                IO_objective = [ip_obj],
                time = [totaltime],
                mptime = [mptime],
                pptime = [sptime],
                iptime = [ip_time],
                cgiter = [cg_iter],
                customers_served = [customers_served],
                customers_transferred = [customers_transferred],
                detour = [detour])
                             
CSV.write(outputfilename, df) #, append=true)
