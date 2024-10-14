
using CSV, Luxor, Colors, Random, DataFrames, Dates, StatsBase

#--------------------------------------------------------------------------------------------------#

thickness = 5
pixelshift = 18

#--------------------------------------------------------------------------------------------------#

function routeviz(drawingname, xdim_full, ydim_full, arcLists, thicknessLists, colorLists, dashLists, colorbyregion)

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

	drawArcList = []	
	for ind in 1:length(arcLists)
		allarcs = arcLists[ind]
		arcthickness, arccolor, arcdash = thicknessLists[ind], colorLists[ind], dashLists[ind]
		for (i,j) in allarcs
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
	if colorbyregion == 1
		regioncolor = ["black", "blue", "red", "green", "violet"]
		for r in 1:numregions, s in locsin[r]
			setcolor(regioncolor[r])
			circle(locationPoints[s], 10, :fill)
		end
	else
		setcolor("black")
		circle.(locationPoints, 10, :fill)
		setcolor("red")
		circle.(hubPoints, 10, :fill)
	end
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

#--------------------------------------------------------------------------------------------------#

function processropt(r_opt, xdim_full, ydim_full)
	
	customerpaths, relevantroutes = Dict(), Dict()
	for j in 1:n
		customerpaths[j] = []
		relevantroutes[j] = []
	end

	subpath = 1
	allvehiclearcs = Dict()
	if referencelines_flag == 1
		for ln in 1:numlines, rr in r_opt[ln]
			vehiclearcs = []
			info = rset[ln][rr]
			if (info.preseta_r != Set()) || (info.presetb1_r != Set()) || (info.presetb2_r != Set())
			
				#Vehicle paths
				for ind in 1:length(info.nodepath_r)-1
					push!(vehiclearcs, (info.nodepath_r[ind], info.nodepath_r[ind+1]))
				end
				allvehiclearcs[subpath] = vehiclearcs

				#Customer driving paths
				for (cust, pickup, dropoff) in info.customer_pudo_r
					pickupind, dropoffind = findfirst(==(pickup),info.nodepath_r), findfirst(==(dropoff),info.nodepath_r)
					custpath = info.nodepath_r[pickupind:dropoffind]
					#Driving arcs along this route
					for ind in 1:length(custpath)-1
						push!(customerpaths[cust], ("drive", custpath[ind], custpath[ind+1]))
						push!(relevantroutes[cust], subpath)
					end
					#Walking arcs along this route
					if cust in info.preseta_r #This route completes this customer's journey (no transfer)
						if oset[cust].o != info.nodepath_r[pickupind]
							push!(customerpaths[cust], ("walk", oset[cust].o, info.nodepath_r[pickupind]))
						end
						if oset[cust].d != info.nodepath_r[dropoffind]
							push!(customerpaths[cust], ("walk", info.nodepath_r[dropoffind], oset[cust].d))
						end
					elseif cust in info.presetb1_r #This route is the first leg of this customer's transfer journey
						if oset[cust].o != info.nodepath_r[pickupind]
							push!(customerpaths[cust], ("walk", oset[cust].o, info.nodepath_r[pickupind]))
						end
					elseif cust in info.presetb2_r #This route is the second leg of this customer's transfer journey
						if oset[cust].d != info.nodepath_r[dropoffind]
							push!(customerpaths[cust], ("walk", info.nodepath_r[dropoffind], oset[cust].d))
						end
					end
				end

				#Visualize
				if viz_flag == 1
					routeviz(string("viz/line",ln,"_route",subpath,".png"), xdim_full, ydim_full, [vehiclearcs], [8], [(0,0,0)], ["solid"])
				end
				subpath += 1

			end
		end
	else
		for rr in r_opt
			vehiclearcs = []
			info = rset[rr]
			if (info.preseta_r != Set()) || (info.presetb1_r != Set()) || (info.presetb2_r != Set())
			
				#Vehicle paths
				for ind in 1:length(info.nodepath_r)-1
					push!(vehiclearcs, (info.nodepath_r[ind], info.nodepath_r[ind+1]))
				end
				allvehiclearcs[subpath] = vehiclearcs
	
				#Customer driving paths
				for (cust, pickup, dropoff) in info.customer_pudo_r
					pickupind, dropoffind = findfirst(==(pickup),info.nodepath_r), findfirst(==(dropoff),info.nodepath_r)
					custpath = info.nodepath_r[pickupind:dropoffind]
					#Driving arcs along this route
					for ind in 1:length(custpath)-1
						push!(customerpaths[cust], ("drive", custpath[ind], custpath[ind+1]))
						push!(relevantroutes[cust], subpath)
					end
					#Walking arcs along this route
					if cust in info.preseta_r #This route completes this customer's journey (no transfer)
						if oset[cust].o != info.nodepath_r[pickupind]
							push!(customerpaths[cust], ("walk", oset[cust].o, info.nodepath_r[pickupind]))
						end
						if oset[cust].d != info.nodepath_r[dropoffind]
							push!(customerpaths[cust], ("walk", info.nodepath_r[dropoffind], oset[cust].d))
						end
					elseif cust in info.presetb1_r #This route is the first leg of this customer's transfer journey
						if oset[cust].o != info.nodepath_r[pickupind]
							push!(customerpaths[cust], ("walk", oset[cust].o, info.nodepath_r[pickupind]))
						end
					elseif cust in info.presetb2_r #This route is the second leg of this customer's transfer journey
						if oset[cust].d != info.nodepath_r[dropoffind]
							push!(customerpaths[cust], ("walk", info.nodepath_r[dropoffind], oset[cust].d))
						end
					end
				end
	
				#Visualize
				if viz_flag == 1
					routeviz(string("viz/route",subpath,".png"), xdim_full, ydim_full, [vehiclearcs], [8], [(0,0,0)], ["solid"])
				end
				subpath += 1
	
			end
		end
	end

	#Customer visualizations
	allhandledorders = []
	orderdistance, bestorderdistance = zeros(n), zeros(n)

	for cust in 1:n
		if customerpaths[cust] != []
			push!(allhandledorders, cust)
			bestorderdistance[cust] += Intersection_car_d_ij[oset[cust].o,oset[cust].d]

			mainroutearcs = []
			for rr in relevantroutes[cust]
				mainroutearcs = union(mainroutearcs, allvehiclearcs[rr])
			end
			walkarcs, drivearcs = [], []
			for (transportmode, orig, dest) in customerpaths[cust]
				if transportmode == "walk"
					push!(walkarcs, (orig,dest))
					orderdistance[cust] += Intersection_car_d_ij[orig,dest]
				elseif transportmode == "drive"
					push!(drivearcs, (orig,dest))
					orderdistance[cust] += Intersection_car_d_ij[orig,dest]
				end
			end

			#Visualize
			if viz_flag == 1
				routeviz(string("viz/customer",cust,".png"), xdim_full, ydim_full, [walkarcs, drivearcs], [7,7], [(200,0,0), (0,150, 255)], ["dashed", "solid"])
			end
		end
	end

	println("Detour = ", round(100* sum(orderdistance[j] for j in allhandledorders; init=0) / sum(bestorderdistance[j] for j in allhandledorders; init=1), digits=2) - 100, "%")

	return sum(orderdistance[j]-bestorderdistance[j] for j in allhandledorders; init=0) / sum(bestorderdistance[j] for j in allhandledorders; init=1) 

end

#--------------------------------------------------------------------------------------------------#

function showsingleroute(drawingname, rr, xdim_full, ydim_full)

	vehiclearcs = []
	info = rset[rr]
	if (info.preseta_r != Set()) || (info.presetb1_r != Set()) || (info.presetb2_r != Set())
	
		#Vehicle paths
		for ind in 1:length(info.nodepath_r)-1
			push!(vehiclearcs, (info.nodepath_r[ind], info.nodepath_r[ind+1]))
		end

		#Visualize
		routeviz(drawingname, xdim_full, ydim_full, [vehiclearcs], [8], [(0,0,0)], ["solid"])
	end

end
