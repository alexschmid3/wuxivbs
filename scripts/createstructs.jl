
struct Order
    o::Int          # origin
    d::Int          # destination
    q::Int          # number of passenger
    g::Float64      # fare
    eb::Float64     # earliest boarding time
    dd::Float64     # deadline
end

#---------------------------------------------------------------------------------------#

"""
Information about a time space arc
"""
struct RouteSet
    start_r::Int                                 # start node
    end_r::Int                                   # end node
    g_r::Float64                                 # arc profit
    preseta_r::Set{Int}                          # direct customer included in this arc >> Mask
    presetb1_r::Set{Int}                         # transfer customer (first trip: pick up to transfer) included in this arc >> Mask
    presetb2_r::Set{Int}                         # transfer customer (second trip: transfer to dropoff) included in this arc >> Mask
    nodepath_r::Array{Int}                       # ordered array of nodes visited
    customer_pudo_r::Array{Tuple{Int64, Int64, Int64}}   # set of tuples = (customer, pickup station, drop off station) for this route
end

#---------------------------------------------------------------------------------------#

"""
Information about a time space arc
"""
struct TempRouteSet
    start_r::Int                                 # start node
    end_r::Int                                   # end node
    g_r::Float64                                 # arc profit
    preseta_r::Set{Int}                          # direct customer included in this arc >> Mask
    presetb1_r::Set{Int}                         # transfer customer (first trip: pick up to transfer) included in this arc >> Mask
    presetb2_r::Set{Int}                         # transfer customer (second trip: transfer to dropoff) included in this arc >> Mask
    nodepath_r::Array{Int}                       # ordered array of nodes visited
    customer_pudo_r::Array{Tuple{Int64, Int64, Int64}}   # set of tuples = (customer, pickup station, drop off station) for this route
    reduced_cost::Float64
end

#---------------------------------------------------------------------------------------#

"""
Information about a status of label-setting dynamic programming
"""
mutable struct LabelSet
    n_l::Int                        # which order pickup/dropoff at the label
    T_l::Float64                    # time at the label
    S_l::Int                        # number of extensions at the label
    Q_l::Int                        # passenger number of the vehicle at the label
    prel_l::Int                     # previous label id
    Dwell_l::Int                    # dwell at a stop
    Transfer_l::Int                 # 1 if is a transfer label here, else 0
    BUS_station_l::Int              # which station
    preset1_ll::Set{Int}            # orders picked up and not dropped off 
    preset2_ll::Set{Int}            # orders have been picked up (cannot be picked up again)
    preset1transfer_ll::Set{Int}    # if this is a transfer pickup, then no transfer dropoff is allowed for this customer. 

    # if pickups conducted, then no dropoff conducted for this stop, because to check capacity, dropoff should be in the first always. 
    BUS_station_continue_dropoff::Set{Int}             # bus station id that not drop off all needed drop-offs // still need to drop off
    BUS_station_start_pickup::Set{Int}             #  this station start pick up (means cannot drop off)
    BUS_station_forbidden::Set{Int}             # bus station id that has pick-up (and these pickups have not been dropped off) in the history path
end

#---------------------------------------------------------------------------------------#

"""
Information about a status of label-setting dynamic programming
"""
mutable struct LabelSet2
    n_l::Int                        # which order pickup/dropoff at the label
    T_l::Float64                    # time at the label
    S_l::Int                        # number of extensions at the label
    Q_l::Int                        # passenger number of the vehicle at the label
    prel_l::Int                     # previous label id
    Dwell_l::Int                    # dwell at a stop
    Transfer_l::Int                 # 1 if is a transfer label here, else 0
    BUS_station_l::Int              # which station
    preset1_ll::Set{Int}            # orders picked up and not dropped off 
    preset2_ll::Set{Int}            # orders have been picked up (cannot be picked up again)
    preset1transfer_ll::Set{Int}    # if this is a transfer pickup, then no transfer dropoff is allowed for this customer. 

    # if pickups conducted, then no dropoff conducted for this stop, because to check capacity, dropoff should be in the first always. 
    BUS_station_continue_dropoff::Set{Int}             # bus station id that not drop off all needed drop-offs // still need to drop off
    BUS_station_start_pickup::Set{Int}             #  this station start pick up (means cannot drop off)
    BUS_station_forbidden::Set{Int}             # bus station id that has pick-up (and these pickups have not been dropped off) in the history path

    #Previous label tracking
    prevlist_l::Array{Int} 
end

#---------------------------------------------------------------------------------------#

"""
Information about a status of label-setting dynamic programming
"""
mutable struct LabelSet3
    n_l::Int                        # which order pickup/dropoff at the label
    T_l::Float64                    # time at the label
    S_l::Int                        # number of extensions at the label
    Q_l::Int                        # passenger number of the vehicle at the label
    prel_l::Int                     # previous label id
    Dwell_l::Int                    # dwell at a stop
    Transfer_l::Int                 # 1 if is a transfer label here, else 0
    BUS_station_l::Int              # which station
    preset1_ll::Set{Int}            # orders picked up and not dropped off 
    preset2_ll::Set{Int}            # orders have been picked up (cannot be picked up again)
    preset1transfer_ll::Set{Int}    # if this is a transfer pickup, then no transfer dropoff is allowed for this customer. 

    # if pickups conducted, then no dropoff conducted for this stop, because to check capacity, dropoff should be in the first always. 
    BUS_station_continue_dropoff::Set{Int}             # bus station id that not drop off all needed drop-offs // still need to drop off
    BUS_station_start_pickup::Set{Int}             #  this station start pick up (means cannot drop off)
    BUS_station_forbidden::Set{Int}             # bus station id that has pick-up (and these pickups have not been dropped off) in the history path

    #Checkpoint tracking
    last_check::Int
    dist_since_last_check::Float64
end