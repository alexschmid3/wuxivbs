
 #Print solution info
function printsolutioninfo_reflines(r_opt)   

    for ln in 1:numlines
        println("--------------- LINE $ln ---------------")
        for rr in r_opt[ln]
            info = rset[ln][rr]
            if (info.preseta_r != Set()) || (info.presetb1_r != Set()) || (info.presetb2_r != Set())
                println("--------- ROUTE $rr ---------")
                println("Start VBS = ", n_location_s[info.start_r])
                println("End VBS = ", n_location_s[info.end_r])
                println("VBS Path = ", info.nodepath_r)
                println("Pick ups and drop offs (cust, pickup station, dropoff station) = ", info.customer_pudo_r)
                for cust in info.preseta_r
                    println("  Customer ", cust, " headed from ", oset[cust].o, " to ", oset[cust].d)
                end
                for ff in info.presetb1_r
                    cust = n_location_f[ff]
                    println("  Customer ", cust, " [TRANSFER 1] headed from ", oset[cust].o, " to ", oset[cust].d)
                end
                for ff in info.presetb2_r
                    cust = n_location_f[ff]
                    println("  Customer ", cust, " [TRANSFER 2] headed from ", oset[cust].o, " to ", oset[cust].d)
                end
            end
        end
    end
end

function printsolutioninfo_noreflines(r_opt)   
    
    for rr in r_opt
        info = rset[rr]
        if (info.preseta_r != Set()) || (info.presetb1_r != Set()) || (info.presetb2_r != Set())
            println("--------- ROUTE $rr ---------")
            println("Start VBS = ", n_location_s[info.start_r])
            println("End VBS = ", n_location_s[info.end_r])
            println("VBS Path = ", info.nodepath_r)
            println("Customer pick ups and drop offs (cust, pickup station, dropoff station) = ", info.customer_pudo_r)
            for cust in info.preseta_r
                println("  Customer ", cust, " headed from ", oset[cust].o, " to ", oset[cust].d)
            end
            for ff in info.presetb1_r
                cust = n_location_f[ff]
                println("  Customer ", cust, " [TRANSFER 1] headed from ", oset[cust].o, " to ", oset[cust].d)
            end
            for ff in info.presetb2_r
                cust = n_location_f[ff]
                println("  Customer ", cust, " [TRANSFER 2] headed from ", oset[cust].o, " to ", oset[cust].d)
            end
        end
    end
        
end

function printsolutioninfo(r_opt)   
    if referencelines_flag == 1
        printsolutioninfo_reflines(r_opt)
    else
        printsolutioninfo_noreflines(r_opt)   
    end
end