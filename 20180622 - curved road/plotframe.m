figure(3)
jj=1;
LaneLength_plot=100;
    Car=Cartemp{jj,1};
    HostID=1;
    [Carhost,Num_CoCar,CoCar,Num_obCar,obCar]=NewRelation(Car,HostID);

             for lane_i=1:1:Num_lane
                if (lane_i==1) || (lane_i==Num_lane)
                    plot([-10,LaneLength_plot],[lane(lane_i),lane(lane_i)],'r-');
                    hold on;      
                else
                    plot([-10,LaneLength_plot],[lane(lane_i),lane(lane_i)],'r--');
                    hold on;
                end
             end
            %画出层级被控车辆
                [plotxx,plotyy]=plotCar(Carhost);
                fill(plotxx,plotyy,'r','edgealpha',0);
                hold on         
            %画出协作车辆
            for ii=1:Num_CoCar
                [plotxx,plotyy]=plotCar(CoCar(ii));
                fill(plotxx,plotyy,'g','edgealpha',0);
                hold on                
            end 
            %画出障碍车辆
            for ii=1:Num_obCar
                [plotxx,plotyy]=plotCar(obCar(ii));
                fill(plotxx,plotyy,'b','edgealpha',0);
                hold on                
            end 
            
            axis equal;
            hold on
            axis([-10 LaneLength_plot lane(1)-2 lane(Num_lane)+2]);
            xlabel('x /m');
            ylabel('y /m');
            pause(0.001);
            hold off
            
   
    