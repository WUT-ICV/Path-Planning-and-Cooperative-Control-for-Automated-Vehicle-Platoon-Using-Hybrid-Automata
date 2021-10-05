%势场参数设置
    HostID=4;
    %设置势场精度
    resolution=0.5; 
    %选择势场时刻
    Moment_t=10;             %1 --- N
for Moment_t=500:20:500
    CarPlot=Cartemp{Moment_t,1};
    [Carhost,Num_CoCar,CoCar,Num_obCar,obCar]=NewRelation(CarPlot,HostID);
    Num_obCartemp=Num_CoCar+Num_obCar;
    obCartemp=[CoCar,obCar];
    
%仿真道路长度
    LaneLength=2*N*T*Carhost.State(4);
    LaneStart=floor(Carhost.State(1)-10);
    LaneEnd=floor(Carhost.State(1)+60);
    [X,Y]=meshgrid(LaneStart:resolution:LaneEnd,Carhost.State(2)-50:resolution:Carhost.State(2)+50);
    [m n]=size(X);%得到显示网格的尺寸
    APF_Value_show=zeros(m,n);
    for n_i=1:1:n
        if n_i==83
            n_i=83;
        end
        for m_i=1:1:m  
            if m_i==14
                m_i=14;
            end
            APF_Value_show(m_i,n_i)=NewAPFPoint_calc(X(m_i,n_i),Y(m_i,n_i),Carhost,Num_obCartemp,obCartemp,Num_lane,lane);
            if APF_Value_show(m_i,n_i)<0
                m_i=m_i;
            end
        end
    end
    APF_Value_show(find(APF_Value_show>15))=15;

figure(1);
    mesh(X,Y,APF_Value_show);    
    hold on
    
    for i_Control=1:Num_Car
        if CarPlot(i_Control).HostFlag==1
            %处理被控车辆
            [Carhost,Num_CoCar,CoCar,Num_obCar,obCar]=NewRelation(CarPlot,i_Control);
            Num_obCartemp=Num_CoCar+Num_obCar;
            obCartemp=[CoCar,obCar];
            
            APFPointValue=NewAPFPoint_calc(Carhost.State(1),Carhost.State(2),Carhost,Num_obCartemp,obCartemp,Num_lane,lane);
            
            plot3(Carhost.State(1),Carhost.State(2),APFPointValue+1.5,'r*');
            
            
            axis equal;
            hold on;
            %axis([0 LaneLength lane(1)-5 lane(Num_lane)+5 ])    
            if i_Control == HostID
                for p=1:1:Np
                    PridictAPFtemp=Carhost.PredictState(p,6);
                    if PridictAPFtemp>20
                        PridictAPFtemp=20;
                    end
                    plot3(Carhost.PredictState(p,1),Carhost.PredictState(p,2),PridictAPFtemp+16,'g*');
                end
            else
                for p=1:1:Np
                    PridictAPFtemp=Carhost.PredictState(p,6);
                    if PridictAPFtemp>20
                        PridictAPFtemp=20;
                    end
                     plot3(Carhost.PredictState(p,1),Carhost.PredictState(p,2),Carhost.PredictState(p,6)+16,'b*');
                end
            end
            
            hold on;
            axis equal;
            axis([LaneStart LaneEnd lane(1)-2 lane(Num_lane)+2 0 37] );
            az = -60;
            el = 25;
            view(az,el);          
        end
    end
    
    pause(0.05);
    hold off;  
    
end