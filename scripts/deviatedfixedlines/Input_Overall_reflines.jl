
using CSV, Luxor, Colors, Random, DataFrames, Dates, StatsBase

#--------------------------------------------------------------------------------------------------#

minthickness, maxthickness = 2,10
pixelshift = 18

#--------------------------------------------------------------------------------------------------#

function determinecoverage(Intersection_car_d_ij)
 
    coverage = Dict()

    for ln in 1:numlines
        coverage[ln] = copy(referencelines[ln])
        for ss in 0:1:maxskipstations, ind in 1:length(referencelines[ln])-1 
            i, j = referencelines[ln][ind], referencelines[ln][mod(ind+ss, length(referencelines[ln]))+1]
            for k in setdiff(1:TOTAL_Station, i, j)
                if Intersection_car_d_ij[i,k] + Intersection_car_d_ij[k,j] - Intersection_car_d_ij[i,j] <= maxdeviation_space*(ss+1)
                    coverage[ln] = union(coverage[ln], k)
                end
            end
        end
    end

	potentialnextstop = Dict()
	for ln in 1:numlines, i in referencelines[ln][1:length(referencelines[ln])-1], k1 in coverage[ln]
		potentialnextstop[ln, i, k1] = []
	end
	for ln in 1:numlines, i in referencelines[ln][1:length(referencelines[ln])-1], j in coverage[ln]
		for ss in 0:1:maxskipstations, ind in 1:length(referencelines[ln])-1 
            i, j = referencelines[ln][ind], referencelines[ln][mod(ind+ss, length(referencelines[ln]))+1]
			for k1 in coverage[ln]
				for k2 in setdiff(coverage[ln], i, k1)
					if Intersection_car_d_ij[i,k1] + Intersection_car_d_ij[k1,k2] + Intersection_car_d_ij[k2,j] - Intersection_car_d_ij[i,j] <= maxdeviation_space*(ss+1)
						push!(potentialnextstop[ln, i, k1], k2)
					end
				end
			end
        end
    end 
	for ln in 1:numlines, i in referencelines[ln][1:length(referencelines[ln])-1], k1 in coverage[ln]
		potentialnextstop[ln, i, k1] = unique(potentialnextstop[ln, i, k1] )
	end
	for ln in 1:numlines, k1 in coverage[ln]
		potentialnextstop[ln, -1, k1] = []
		for i in referencelines[ln][1:length(referencelines[ln])-1]
			potentialnextstop[ln, -1, k1] = union(potentialnextstop[ln, -1, k1], potentialnextstop[ln, i, k1])
		end
	end

    return coverage, potentialnextstop

end

#---------------------------------------------------------------------------------------#

function visualizecoverage(drawingname, xdim_full, ydim_full)

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

    rval = Dict(1 => 0,   2 => 243, 3 => 206, 4 => 110)
    gval = Dict(1 => 113, 2 => 97,  3 => 14,  4 => 72)
    bval = Dict(1 => 213, 2 => 32,  3 => 118, 4 => 154)

    drawArcList = []	
    for ln in 1:4
        for ss in 0:1:maxskipstations, ind in 1:length(referencelines[ln])-1  
            i, j = referencelines[ln][ind], referencelines[ln][ind+1]
            for k in setdiff(1:TOTAL_Station, i, j)
                if Intersection_car_d_ij[i,k] + Intersection_car_d_ij[k,j] - Intersection_car_d_ij[i,j] <= maxdeviation_space*(ss+1)
                    arcthickness = 4 
                    arccolor = ((rval[ln]+255)/2, (gval[ln]+255)/2, (bval[ln]+255)/2)
                    arcdash = "solid"
                    startPoint1 = locationPoints[i]
                    endPoint1 = locationPoints[k]
                    push!(drawArcList, (startPoint1, endPoint1, arccolor, arcthickness, arcdash))
                    startPoint2 = locationPoints[k]
                    endPoint2 = locationPoints[j]
                    push!(drawArcList, (startPoint2, endPoint2, arccolor, arcthickness, arcdash))
                end
            end
        end
    end

    for ln in 1:numlines, ind in 1:length(referencelines[ln])-1
        i, j = referencelines[ln][ind], referencelines[ln][ind+1]
        arcthickness = 10 
        arccolor = (rval[ln], gval[ln], bval[ln])
        arcdash = "solid"
        startPoint = locationPoints[i]
        endPoint = locationPoints[j]
        push!(drawArcList, (startPoint, endPoint, arccolor, arcthickness, arcdash))
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
	setcolor(convert(Colors.HSV, Colors.RGB(251/255,189/255,66/255)))
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

