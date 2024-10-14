
function input_data_Manhattan_Intersection_Coordinates_Mod_Dis_Car(TOTAL_Station::Int)
    Intersection_car_d_ij = zeros(Float64, TOTAL_Station + 2, TOTAL_Station + 2)
    open(distcarfilename) do dist_reader
        readline(dist_reader)
        while !eof(dist_reader)
            line = readline(dist_reader)
            if isempty(line)
                continue
            end
            values = split(line, ',')
            begin_i = parse(Int, values[1])
            end_i = parse(Int, values[2])
            dist = parse(Float64, values[3])
            
            Intersection_car_d_ij[begin_i, end_i] = dist
        end
    end
    return Intersection_car_d_ij
end

#---------------------------------------------------------------------------------------#

function Bus_Station_Input_real(Intersection_car_d_ij::Array)
    Bus_station_travel_time = zeros(Float64, TOTAL_Station + 2, TOTAL_Station + 2)
    HUB_station = Set{Int}()
    station_coordinates = zeros(Float64, TOTAL_Station, 2)
    open(nodesfilename) do node_reader
        readline(node_reader) # skip first line (header)
        cc1 = 1
        while !eof(node_reader)
            line = readline(node_reader)
            if isempty(line)
                continue
            end
            values = split(line, ',')
            index = parse(Int, values[1])
            station_coordinates[index,1] = parse(Float64, values[2])
            station_coordinates[index,2] = parse(Float64, values[3])

            if index in manualhubs  # cc1 * 5
                push!(HUB_station, index)
                cc1 += 1
            end
        end
    end
    for stop1 in 1:TOTAL_Station + 2
        for stop2 in 1:TOTAL_Station + 2
            Bus_station_travel_time[stop1, stop2] = Intersection_car_d_ij[stop1, stop2] / sv
        end
    end
    return HUB_station, Bus_station_travel_time, station_coordinates
end


#---------------------------------------------------------------------------------------#

"""
Function to get alternative near stations of each station
"""
#VCC options
function input_customer_intersection_relationship(discre_no::Int)

    # Read data from dist-pedestrian-small.csv
    match_reader = open(distpedestrianfilename, "r")
    match_no = zeros(Int, TOTAL_Station + 1, discre_no) #zeros(Int, TOTAL_Station + 1, discre_no + 1)
    readline(match_reader) # skip first line (header)
    while !eof(match_reader)
        line = readline(match_reader)
        if isempty(line)
            continue
        end
        values = split(line, ',')
        begin_i = parse(Int, values[1])
        cad_i = parse(Int, values[2]) #Ranking of how close stop end_i is to stop begin_i (e.g. 1 if begin_i=end_i)
        end_i = parse(Int, values[3])
        if cad_i <= discre_no #Only allow customers to walk to VBS if it is one of the k closest stops, where k = discre_no (?)
            match_no[begin_i, cad_i] = end_i
        end
    end
    close(match_reader)
    return match_no
end

#---------------------------------------------------------------------------------------#

"""
Function to input and process customer orders

### input
* `control` - if 1 transfer, 0 not transfer
* `discre_no` - number of possible matching pick/drop stations
* `previous_resolve_start` - start collecting orders time
* `resolve_start` - end collecting orders time
* `Intersection_car_d_ij` - intersection coordinates (car distance data)
* `HUB_station` - transfer hub data
* `match_no` - alternative near stations of each station
"""

function Request_Input_real(control::Int, discre_no::Int, previous_resolve_start::Int, resolve_start::Int, Intersection_car_d_ij::Array, HUB_station::Set, match_no::Array, Bus_station_travel_time::Array)
  
    # Initialize variables
    index = 1
    # Open file for reading
    filedata = CSV.read(customerfilename, DataFrame)
    for ln in 1:size(filedata)[1]   
        if customerfilename == "data/customers/order-1218-hs-small.csv"
            dateTime1 = DateTime(filedata[ln,2], "yyyy-mm-dd HH:MM:SS")
        else 
            dateTime1 = DateTime(filedata[ln,2])
        end
        timeSpan1 = dateTime1 - DateTime(2023, 12, 18, 6, 30, 0) #, tz = UTC)
        request_time = Int(floor(Dates.value(timeSpan1) / 1000))
        
        if (request_time <= resolve_start) && (request_time > previous_resolve_start)
            begin_i = filedata[ln,8]
            end_i = filedata[ln,9]
            q_i = filedata[ln,3]
            g_i = Intersection_car_d_ij[begin_i, end_i] / 1000.0 * 0.86 * q_i
            dd_i = request_time + Intersection_car_d_ij[begin_i, end_i] / sv * 2
            order = Order(begin_i, end_i, q_i, g_i, request_time, dd_i)
            #oset[index] = order
            push!(oset, order)
            index += 1
        end
        if (request_time > resolve_start)
            break
        end

    end

    n = length(oset)
    
    #---------------------------------------------------------#

    # Initialize arrays and perform operations
    Possiblematch_station_o = [Set{Int}() for _ in 1:n]
    Possiblematch_station_d = [Set{Int}() for _ in 1:n]
    Hub_Possiblematch_station = [Set{Int}() for _ in 1:n]
    for jj in 1:n
        order = oset[jj]
        Possiblematch_station_o[jj] = Set{Int}()
        Possiblematch_station_d[jj] = Set{Int}()
        #Add all stations that are within walking distance (as defined by discre_no, i.e. vcc_candidates)
        if vcc_type == "candidates"    
            for i in 1:discre_no
                push!(Possiblematch_station_o[jj], match_no[order.o, i])
                push!(Possiblematch_station_d[jj], match_no[order.d, i])
            end
        elseif vcc_type == "distance"
            orig, dest = order.o, order.d
            for station in 1:TOTAL_Station
                if Intersection_car_d_ij[orig,station] <= vcc_maxdist
                    push!(Possiblematch_station_o[jj], station)
                end
                if Intersection_car_d_ij[dest,station] <= vcc_maxdist
                    push!(Possiblematch_station_d[jj], station)
                end
            end
        end
        for stopXX in HUB_station
            if !(stopXX in Possiblematch_station_o[jj]) && !(stopXX in Possiblematch_station_d[jj])
                if Intersection_car_d_ij[order.o, order.d] * transfer_maxdev >= Intersection_car_d_ij[order.o, stopXX] + Intersection_car_d_ij[stopXX, order.d]
                    if length(Hub_Possiblematch_station[jj]) < transfer_maxhubs
                        push!(Hub_Possiblematch_station[jj], stopXX)
                    else
                        break
                    end
                end
            end
        end
    end

    if control == 0
        for jj in 1:n
            Hub_Possiblematch_station[jj] = Set{Int}()
        end
    end

    # Shortest transfer duration
    shortest_arrive_transfer_duration_jstation = zeros(Float64, n + 1, TOTAL_Station + 2) # Request_Input: earliestboarding
    shortest_arrive_destination_duration_jstation = zeros(Float64, n + 1, TOTAL_Station + 2) # Request_Input: earliestboarding
    shortest_direct_arrive_destination_duration_jstation = zeros(Float64, n + 1, TOTAL_Station + 2) # Request_Input: earliestboarding

    for jj in 1:n
        for stopXXhub in Hub_Possiblematch_station[jj]
            shortest_arrive_transfer_duration_jstation[jj, stopXXhub] = typemax(Int)
            for stopXXorigin in Possiblematch_station_o[jj]
                if shortest_arrive_transfer_duration_jstation[jj, stopXXhub] > Bus_station_travel_time[stopXXorigin, stopXXhub]
                    shortest_arrive_transfer_duration_jstation[jj, stopXXhub] = Bus_station_travel_time[stopXXorigin, stopXXhub]
                end
            end
            shortest_arrive_destination_duration_jstation[jj, stopXXhub] = typemax(Int)
            for stopXXdestination in Possiblematch_station_d[jj]
                if shortest_arrive_destination_duration_jstation[jj, stopXXhub] > Bus_station_travel_time[stopXXhub, stopXXdestination]
                    shortest_arrive_destination_duration_jstation[jj, stopXXhub] = Bus_station_travel_time[stopXXhub, stopXXdestination]
                end
            end
        end
    end

    # Shortest direct arrive duration
    for jj in 1:n
        for stopXXorigin in Possiblematch_station_o[jj]
            shortest_direct_arrive_destination_duration_jstation[jj, stopXXorigin] = typemax(Int)
            for stopXXdestination in Possiblematch_station_d[jj]
                if shortest_direct_arrive_destination_duration_jstation[jj, stopXXorigin] > Bus_station_travel_time[stopXXorigin, stopXXdestination]
                    shortest_direct_arrive_destination_duration_jstation[jj, stopXXorigin] = Bus_station_travel_time[stopXXorigin, stopXXdestination]
                end
            end
        end
    end

    return n, Possiblematch_station_o, Possiblematch_station_d, Hub_Possiblematch_station, shortest_arrive_transfer_duration_jstation, shortest_arrive_destination_duration_jstation, shortest_direct_arrive_destination_duration_jstation

end

#---------------------------------------------------------------------------------------#

"""
Function to get mapping from station to related orders 
    (possible matching pick-up/drop-off for each station)
"""
function Station_request_matching(n::Int, Possiblematch_station_o::Vector{Set{Int64}}, Possiblematch_station_d::Vector{Set{Int64}}, Hub_Possiblematch_station::Vector{Set{Int64}})
        
    prelate_j_station = [Set{Int}() for _ in 1:TOTAL_Station]
    drelate_j_station = [Set{Int}() for _ in 1:TOTAL_Station]
  
    for jj in 1:n  
        for stopXX in Possiblematch_station_o[jj]
            push!(prelate_j_station[stopXX], jj)
        end
        for stopXX in Possiblematch_station_d[jj]
            push!(drelate_j_station[stopXX], jj + n)
        end
        for stopXX in Hub_Possiblematch_station[jj]
            push!(prelate_j_station[stopXX], jj)
            push!(drelate_j_station[stopXX], jj + n)
        end
    end

    return prelate_j_station, drelate_j_station
end

#---------------------------------------------------------------------------------------#

"""
Function to input all information

### input
* `control` - if 1 transfer, 0 not transfer
* `discre_no` - number of possible matching pick/drop stations
* `previous_resolve_start` - start collecting orders time
* `resolve_start` - end collecting orders time
* `Intersection_car_d_ij` - intersection coordinates (car distance data)
* `HUB_station` - transfer hub data
* `match_no` - alternative near stations of each station
"""
function Input_Overall(control::Int, discre_no::Int)
    Intersection_car_d_ij = input_data_Manhattan_Intersection_Coordinates_Mod_Dis_Car(TOTAL_Station)
    HUB_station, Bus_station_travel_time, station_coordinates = Bus_Station_Input_real(Intersection_car_d_ij)  # Input: bus station information
    match_no = input_customer_intersection_relationship(discre_no)
    n, Possiblematch_station_o, Possiblematch_station_d, Hub_Possiblematch_station, shortest_arrive_transfer_duration_jstation, shortest_arrive_destination_duration_jstation, shortest_direct_arrive_destination_duration_jstation = Request_Input_real(control, discre_no, previous_resolve_start, resolve_start, Intersection_car_d_ij, HUB_station, match_no, Bus_station_travel_time)  # Input: passenger requests
    prelate_j_station, drelate_j_station = Station_request_matching(n, Possiblematch_station_o, Possiblematch_station_d, Hub_Possiblematch_station)

    if referencelines_flag == 1
        coverage, potentialnextstop = determinecoverage(Intersection_car_d_ij)
    else
        coverage = false
        potentialnextstop = false
    end

    return n, Bus_station_travel_time, station_coordinates, Possiblematch_station_o, Possiblematch_station_d, Hub_Possiblematch_station, shortest_arrive_transfer_duration_jstation, shortest_arrive_destination_duration_jstation, shortest_direct_arrive_destination_duration_jstation, prelate_j_station, drelate_j_station, match_no, Intersection_car_d_ij, HUB_station, coverage, potentialnextstop

end