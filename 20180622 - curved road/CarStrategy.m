function Car = CarStrategy(Car,HostID)
%UNTITLED Summary of this function goes here


    [Carhost,Num_CoCar,CoCar,Num_obCar,obCar]=NewRelation(Car,HostID);    
    
    %被控车辆策略检测

        if Carhost.platoon == Carhost.ID && Num_CoCar~=0 && Carhost.PltnEn == 1
            potentialfollowNum = 0;
            LeaderNum = 0;
            for St_i=1:1:Num_CoCar       % St_i short for Strategy i                
                if ((CoCar(St_i).Roadmilestore(1) > Carhost.Roadmilestore(1)+5 )&&( CoCar(St_i).Roadmilestore(1) - Carhost.Roadmilestore(1)<= 20 )...
                    &&(abs(CoCar(St_i).RoadLocation(2)-Carhost.RoadLocation(2))<=1)) && CoCar(St_i).PltnLength == CoCar(St_i).PltnNum
                    %找到前方潜在跟随车辆中横向距离最短的一辆
                    
                   
                    
                    potentialfollowNum = potentialfollowNum+1;                    
                    Temp_yerror = abs(CoCar(St_i).State(2)-Carhost.State(2));   % 依次查询被控车辆前方协作车辆与
                    Temp_yerror = abs((Carhost.State(2)-CoCar(St_i).State(2))*cos(CoCar(St_i).State(3))-(Carhost.State(1)-CoCar(St_i).State(1))*sin(CoCar(St_i).State(3)));
                    if potentialfollowNum ==1 || Min_yerror>= Temp_yerror
                        Min_yerror=Temp_yerror;
                        LeaderNum=St_i;
                    end
                end
            end
            if LeaderNum~=0
                for pltn_i=1:1:Num_CoCar
                    if CoCar(pltn_i).platoon == Carhost.platoon
                        Car(CoCar(pltn_i).ID).platoon = CoCar(LeaderNum).platoon;
                        Car(CoCar(pltn_i).ID).PltnNum = Car(CoCar(pltn_i).ID).PltnNum+CoCar(LeaderNum).PltnNum;
                    end
                end
                Car(HostID).platoon = CoCar(LeaderNum).platoon;
                Car(HostID).LeaderID = CoCar(LeaderNum).ID;
                Car(HostID).PltnNum = CoCar(LeaderNum).PltnNum+1;
                for pltn_i=1:1:Num_CoCar
                    if CoCar(pltn_i).platoon == CoCar(LeaderNum).platoon 
                        Car(CoCar(pltn_i).ID).PltnLength = Carhost.PltnLength+CoCar(LeaderNum).PltnLength;
                    elseif CoCar(pltn_i).platoon == Carhost.platoon 
                        Car(CoCar(pltn_i).ID).PltnLength = Carhost.PltnLength+CoCar(LeaderNum).PltnLength;
                    end
                end
                Car(HostID).PltnLength = Carhost.PltnLength+CoCar(LeaderNum).PltnLength;
            end
     
        elseif Carhost.platoon ~= Carhost.ID
            if (abs(Car(Carhost.LeaderID).Roadmilestore(1)-Carhost.Roadmilestore(1))>25 && Car(Carhost.LeaderID).State(4)> Carhost.State(4))...
               || Carhost.PltnEn == 0 %...
          %     || abs(Car(Carhost.LeaderID).State(2)-Carhost.State(2))>3.2
                Car(HostID).intraDis=30;
                if abs(Car(Carhost.LeaderID).Roadmilestore(1)-Carhost.Roadmilestore(1))>Car(HostID).intraDis-5
                    for pltn_i=1:1:Num_CoCar
                        if (CoCar(pltn_i).platoon == Carhost.platoon && CoCar(pltn_i).PltnNum>Carhost.PltnNum)
                            Car(CoCar(pltn_i).ID).platoon = Carhost.ID;
                            Car(CoCar(pltn_i).ID).PltnNum = Car(CoCar(pltn_i).ID).PltnNum-Carhost.PltnNum+1;
                            Car(CoCar(pltn_i).ID).PltnLength = Carhost.PltnLength - Carhost.PltnNum+1;
                        elseif (CoCar(pltn_i).platoon == Carhost.platoon && CoCar(pltn_i).PltnNum < Carhost.PltnNum)
                            Car(CoCar(pltn_i).ID).PltnLength =  Carhost.PltnNum - 1;
                        end
                    end
                        Car(HostID).platoon = Car(HostID).ID;
                        Car(HostID).LeaderID = Car(HostID).ID;
                        Car(HostID).PltnNum = 1;       
                        Car(HostID).PltnLength = Carhost.PltnLength - Carhost.PltnNum+1;
                        Car(HostID).intraDis=15;
                end                
                
            end
            
        end


end