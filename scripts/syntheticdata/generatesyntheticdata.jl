
using PoissonRandom

const TOTAL_Station = 65 
const targetordersper10min = 250

basecustomerfilename = "data/order-1218-hs-small.csv" 
newcustomerfilename = string("data/order-syn-n",targetordersper10min,".csv")

data = CSV.read(basecustomerfilename, DataFrame)

#Distance between locs
Intersection_car_d_ij = input_data_Manhattan_Intersection_Coordinates_Mod_Dis_Car(TOTAL_Station)

#Read in original data 
origindestdist = zeros(TOTAL_Station, TOTAL_Station)
quantityappearances = []
for line in 1:size(data)[1]
    begin_i = data[line,8]
    end_i = data[line,9]
    q_i = data[line,3]
    origindestdist[begin_i, end_i] += q_i
    push!(quantityappearances, q_i)
end

#Add in some probability to other o-d pairs
for i in 1:TOTAL_Station, j in setdiff(1:TOTAL_Station,i)
    if Intersection_car_d_ij[i,j] > 1200
        origindestdist[i,j] += 0.5
    end 
end

#Normalize
origindestdist /= sum(origindestdist)
cumul, cumullookup = [], []
for i in 1:TOTAL_Station, j in 1:TOTAL_Station
    sofar = sum(sum(origindestdist[i2,j2] for j2 in 1:TOTAL_Station, init=0) for i2 in 1:i-1; init=0) + sum(origindestdist[i,j2] for j2 in 1:j; init=0)
    push!(cumul, sofar)
    push!(cumullookup, (i,j))
end

#Get original Poisson paramters for new order arrivals
arrivaltimes = []
lastdatetime = DateTime(2023, 12, 18, 6, 30, 0)
for line in 1:size(data)[1]
    interarrivaltime =  DateTime(data[line,2], "yyyy-mm-dd HH:MM") - lastdatetime
    push!(arrivaltimes, interarrivaltime)

    lastdatetime = DateTime(data[line,2], "yyyy-mm-dd HH:MM")
end
λ_orig = (sum(arrivaltimes).value / 1000) / length(arrivaltimes)

#Generate a new file
orderindex = 1
λ = 600 / targetordersper10min
currentdatetime = DateTime(2023, 12, 18, 6, 30, 0) + Dates.Second(pois_rand(λ))
idlist, ordertimelist, countlist, busidlist, paylist, gainlist, statuslist, pickupnodelist, dropoffnodelist = [], [], [], [], [], [], [], [], []
while currentdatetime <=  DateTime(2023, 12, 18, 21, 30, 0)
    #Draw a new order from the distribution
    randomindex = rand()
    (newo,newd) = cumullookup[length([item for item in cumul if item < randomindex])+1]
    q_i = rand(quantityappearances)
    g_i = Intersection_car_d_ij[newo, newd] / 1000.0 * 0.86 

    #Add to filelists
    push!(idlist, orderindex)
    push!(ordertimelist, DateTime(Dates.Year(currentdatetime),Dates.Month(currentdatetime),Dates.Day(currentdatetime),Dates.Hour(currentdatetime),Dates.Minute(currentdatetime)))
    push!(countlist, q_i)
    push!(busidlist, -1)
    push!(paylist, -1)
    push!(gainlist, g_i)
    push!(statuslist, -1)
    push!(pickupnodelist, newo)
    push!(dropoffnodelist, newd)

    orderindex += 1
    currentdatetime = currentdatetime + Dates.Second(convert(Int,round(pois_rand(λ),digits=0)))
end

df = DataFrame(
        id = idlist,	
        order_time = ordertimelist,
        count = countlist,
        bus_id = busidlist,
        actual_pay = paylist,
        actual_gain = gainlist,
        status = statuslist,
        pickup_node = pickupnodelist,
        dropoff_node = dropoffnodelist
    )

CSV.write(newcustomerfilename, df)