
function divideintoregions(numregions::Int)

    model = Model(Gurobi.Optimizer)
    set_optimizer_attribute(model, "TimeLimit", 60)  # 2 hours time limit
    set_optimizer_attribute(model, "MIPGap", 0.01)  # Relative MIP optimality gap

    minassigned = floor(TOTAL_Station/numregions)
    maxassigned = ceil(TOTAL_Station/numregions)
   
    # Variables
    @variable(model, x[1:TOTAL_Station, 1:numregions], Bin)
    @variable(model, y[1:TOTAL_Station, 1:TOTAL_Station], Bin)

    # Constraints
    @constraint(model, oneperloc[s = 1:TOTAL_Station], sum(x[s,r] for r in 1:numregions) == 1)
    @constraint(model, totalassigned[r = 1:numregions], minassigned <= sum(x[s,r] for s in 1:TOTAL_Station) <= maxassigned)
    @constraint(model, assignedtosameregion[s1 in 1:TOTAL_Station, s2 in 1:TOTAL_Station, r in 1:numregions], y[s1,s2] >= x[s1,r] + x[s2,r] - 1)

    # objective
    @objective(model, Min, sum(sum(Intersection_car_d_ij[s1,s2] * y[s1,s2] for s2 in s1+1:TOTAL_Station) for s1 in 1:TOTAL_Station))

    optimize!(model)

    locsin, regionof = Dict(), Dict()
    for r in 1:numregions
        locsin[r] = []
    end
    for s in 1:TOTAL_Station, r in 1:numregions
        if value(x[s,r]) > 1e-4
            push!(locsin[r], s)
            regionof[s] = r
        end
    end

    return locsin, regionof

end