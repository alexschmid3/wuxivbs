
using CSV, Luxor, Colors, Random, DataFrames, Dates, StatsBase

#--------------------------------------------------------------------------------------------------#

minthickness, maxthickness = 2,10
pixelshift = 18

#--------------------------------------------------------------------------------------------------#

function routeviz(drawingname, xdim_full, ydim_full)

    xdim, ydim = xdim_full - 100, ydim_full - 100

    #Get correct scale
	maxlat, minlat = maximum(station_coordinates[:,1]), minimum(station_coordinates[:,1])
	maxlong, minlong = maximum(station_coordinates[:,2]), minimum(station_coordinates[:,2])
    
    #latmult = -(xdim-200) / (maxlat - minlat)	 
    #latshift = -(xdim-200)/2 + (xdim-200) * maxlat / (maxlat - minlat)
	#longmult = -1 * latmult * 24/29
	#longshift = -(maxlongcoord + minlongcoord)/2

    latmult = min(ydim / (maxlat-minlat), xdim / (maxlong-minlong))
    longmult = latmult
    latshift = (maxlat + minlat)/2 * latmult * -1
    longshift = (maxlong + minlong)/2 * longmult * -1
	 
	#Format and transform latitude and longitude coordinates of each pit stop
	pointDict = Dict()
	listofpoints, listofhubs = [], []
	listofpoints_labels = []
	for l in 1:size(station_coordinates)[1]
		longitude, latitude = station_coordinates[l,2], station_coordinates[l,1]
		transformedcoords = (longmult*longitude+longshift, latmult*latitude+latshift)
		pointDict[l] = Point(transformedcoords)
		push!(listofpoints, transformedcoords)
		push!(listofpoints_labels, [transformedcoords, string(l)])
		if l in HUB_station
			push!(listofhubs, transformedcoords)
		end
	end
	locationPoints = Point.(listofpoints)
	hubPoints = Point.(listofhubs)

    #--------------------------------------------------------#

    #Read in arcs
    totaltraffic = zeros(TOTAL_Station, TOTAL_Station)
    odf = CSV.read(customerfilename, DataFrame)
    for row in 1:size(odf)[1]
        orig,dest = odf[row,8], odf[row,9]
        totaltraffic[orig,dest] += 1
    end
    mosttraffic, leasttraffic = maximum(totaltraffic), 1

    #--------------------------------------------------------#

    orders = []
    for i in 1:TOTAL_Station, j in 1:TOTAL_Station
        if totaltraffic[i,j] > 1e-4
            push!(orders, (i,j))
        end
    end

    validarcs = []
    arcsfrom, arcsto = Dict(), Dict()
    for i in 1:TOTAL_Station
        arcsfrom[i] = []
        arcsto[i] = []
    end
    for i in 1:TOTAL_Station, j in 1:TOTAL_Station
        if Intersection_car_d_ij[i,j] <= 2000
            push!(validarcs, (i,j))
            push!(arcsfrom[i], j)
            push!(arcsto[j], i)
        end
    end
    
    #--------------------------------------------------------#

    model = Model(Gurobi.Optimizer)
    @variable(model, x[i in 1:TOTAL_Station, j in arcsfrom[i]], Bin)
    @variable(model, y[orders, i in 1:TOTAL_Station, j in arcsfrom[i]], Bin)
    
    #@objective(model, Max, sum(sum(totaltraffic[i,j] * y[(i,j),i2,j] for i2 in arcsto[j]) for (i,j) in orders))
    #@objective(model, Min, sum(sum(sum(totaltraffic[i,j] * Intersection_car_d_ij[i2,j2] * y[(i,j),i2,j2] for j2 in arcsfrom[i2]) for i2 in 1:TOTAL_Station) for (i,j) in orders))
    @objective(model, Min, sum(Intersection_car_d_ij[i,j] * x[i,j] for (i,j) in validarcs))

    #@constraint(model, maxdistance, sum(Intersection_car_d_ij[i,j] * x[i,j] for (i,j) in validarcs) <= 8000*5)
    @constraint(model, linking[(i,j) in orders,(i2,j2) in validarcs], y[(i,j),i2,j2] <= x[i2,j2])
    @constraint(model, flowbalance[(i,j) in orders, i2=setdiff(1:TOTAL_Station,i,j)], sum(y[(i,j),i2,j2] for j2 in arcsfrom[i2]) - sum(y[(i,j),j2,i2] for j2 in arcsto[i2]) == 0)
    @constraint(model, pickupall[(i,j) in orders, i2=setdiff(1:TOTAL_Station,i,j)], sum(y[(i,j),i,j2] for j2 in arcsfrom[i]) == 1)
    @constraint(model, deliverall[(i,j) in orders, i2=setdiff(1:TOTAL_Station,i,j)], sum(y[(i,j),i2,j] for i2 in arcsto[j]) == 1)
    @constraint(model, settozero[(i,j) in orders, i2=1:TOTAL_Station], y[(i,j),i2,i2] == 0)
    @constraint(model, cantreturntoorig[(i,j) in orders], sum(y[(i,j),j2,i] for j2 in arcsto[i]) == 0)
    @constraint(model, cantleavedest[(i,j) in orders], sum(y[(i,j),j,i2] for i2 in arcsfrom[j]) == 0)

    optimize!(model)
    objective_value(model)

    #--------------------------------------------------------#

	#=drawArcList = []	
	for i in 1:TOTAL_Station, j in 1:TOTAL_Station
		arcthickness = (totaltraffic[i,j] / mosttraffic) * (maxthickness - minthickness) + minthickness
        arccolor = ((1 - (totaltraffic[i,j] / mosttraffic)) * 250,(1 - (totaltraffic[i,j] / mosttraffic)) * 250,(1 - (totaltraffic[i,j] / mosttraffic)) * 250)
        arcdash = "solid"
		startPoint = locationPoints[i]
		endPoint = locationPoints[j]
		push!(drawArcList, (startPoint, endPoint, arccolor, arcthickness, arcdash))
	end=#
    maxtraffic=0
    for (i,j) in validarcs
        maxtraffic = max(maxtraffic, sum(totaltraffic[i2,j2] * value(y[(i2,j2),i,j]) for (i2,j2) in orders))
    end
    drawArcList = []	
	for (i,j) in validarcs
        if value(x[i,j]) > 1e-4
            arcthickness = 10 #(totaltraffic[i,j] / mosttraffic) * (maxthickness - minthickness) + minthickness
            arccolor = ((1 - (sum(totaltraffic[i2,j2] * value(y[(i2,j2),i,j]) for (i2,j2) in orders) / maxtraffic)) * 220,(1 - (sum(totaltraffic[i2,j2] * value(y[(i2,j2),i,j]) for (i2,j2) in orders) / maxtraffic)) * 220,(1 - (sum(totaltraffic[i2,j2] * value(y[(i2,j2),i,j]) for (i2,j2) in orders) / maxtraffic)) * 220)
            arcdash = "solid"
            startPoint = locationPoints[i]
            endPoint = locationPoints[j]
            push!(drawArcList, (startPoint, endPoint, arccolor, arcthickness, arcdash))
        end
	end
		
	#--------------------------------------------------------#

	#Create new drawing
	Drawing(xdim_full, ydim_full, drawingname)
	origin()
	background("white")

	#Draw the arcs
	for i in drawArcList
		#Set arc attributes
		setline(i[4])
		#setcolor(i[3])
		r_val, g_val, b_val = i[3][1]/255, i[3][2]/255, i[3][3]/255
		setcolor(convert(Colors.HSV, Colors.RGB(r_val, g_val, b_val)))  #You can also use setcolor("colorname")
		setdash(i[5])

		#Draw the arc line
		line(i[1], i[2] , :stroke)
		
		#Calculate the rotation and placement of the arrowhead
		theta = atan((i[2][2] - i[1][2])/(i[2][1] - i[1][1]))
		dist = distance(i[1], i[2])
		arrowhead = (1-pixelshift/dist)*i[2] + (pixelshift/dist)*i[1] #center of arrowhead positioned 8 pixels from the end node

		#Rotate the arrowhead appropriately
		if i[1][1] >= i[2][1]
			local p = ngon(arrowhead, min(pixelshift, i[4]*2), 3, theta - pi , vertices=true)
		else
			local p = ngon(arrowhead, min(pixelshift, i[4]*2), 3, theta , vertices=true)
		end

		#Draw the arrowhead
		poly(p, :fill,  close=true)
	end

	#Draw the pit stop nodes
	setcolor("black")
	circle.(locationPoints, 10, :fill)
	setcolor("red")
	circle.(hubPoints, 10, :fill)
    setcolor("black")
    setline(3)
    circle.(locationPoints, 10, :stroke)

	#Add pit stop labels
	fontsize(22)
	setcolor("black")
	for item in listofpoints_labels
 		#label(item[2], :0, Point(item[1]))
		Luxor.text(item[2],  Point(item[1])+Point((0,-23)), halign=:center, valign = :middle)
	end
	setcolor("black")

	#--------------------------------------------------------#

	finish()
	preview()

end
