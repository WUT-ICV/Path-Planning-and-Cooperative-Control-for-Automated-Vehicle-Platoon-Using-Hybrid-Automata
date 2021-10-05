%车道参数初始化设置（这里假设车道线是直线，平行于x轴）
Num_lane=4;   %车道线数量
lane=zeros(Num_lane,1);
lane(1)=-3;   %车道线纵坐标
lane(2)=0;    %车道线纵坐标
lane(3)=3;    %车道线纵坐标
lane(4)=6;    %车道线纵坐标

Car(1).State=[0      1.5   0    22     0];     % x y phi v WheelAng（前轮偏角）
Car(2).State=[40     1.5   0    10     0];     % x y phi v WheelAng（前轮偏角）

[temp_m,Num_Car]=size(Car);
%初始化车辆参数为默认值
for temp_i=1:Num_Car
    Car(temp_i).length=4.8;
    Car(temp_i).width=1.8;
    Car(temp_i).platoon=temp_i;
    Car(temp_i).PltnNum=1;
    Car(temp_i).ID=temp_i;    
    Car(temp_i).comEn=0;    
    Car(temp_i).HostFlag=0;
    Car(temp_i).SpeedSet=Car(temp_i).State(4);
    Car(temp_i).PredictState=[];
    Car(temp_i).a_real=[];  
    Car(temp_i).deltaf_real=[];
    Car(temp_i).v_real=[];
    Car(temp_i).wheelang_real=[];
    Car(temp_i).SensorRange=50;
    Car(temp_i).WirelessRange=200;    
end

Car(1).HostFlag=1;

%势场参数设置
    HostID=1;
    %设置势场精度
    resolution=0.1; 

    CarPlot=Car;
    [Carhost,Num_CoCar,CoCar,Num_obCar,obCar]=NewRelation(CarPlot,HostID);
    Num_obCartemp=Num_CoCar+Num_obCar;
    obCartemp=[CoCar,obCar];
    
%仿真道路长度

    LaneStart=floor(Carhost.State(1)-10);
    LaneEnd=floor(Carhost.State(1)+60);
    [X,Y]=meshgrid(LaneStart:resolution:LaneEnd,lane(1)-5:resolution:lane(Num_lane)+5);
    [m n]=size(X);%得到显示网格的尺寸
    APF_Value_show=zeros(m,n);
    for n_i=1:1:n
        for m_i=1:1:m  
            APF_Value_show(m_i,n_i)=NewAPFPoint_calc(X(m_i,n_i),Y(m_i,n_i),Carhost,Num_obCartemp,obCartemp,Num_lane,lane);
            if APF_Value_show(m_i,n_i)<0
                m_i=m_i;
            end
        end
    end
    APF_Value_show(find(APF_Value_show>15))=15;

% figure(3);
%     mesh(X,Y,APF_Value_show);    
%     hidden on
%     hold on
%     axis equal;
%     axis([LaneStart LaneEnd lane(1)-2 lane(Num_lane)+2 0 15] );
%     az = -60;
%     el = 25;
%     view(az,el);   
%     
figure(2)
    plot(Y(:,1),APF_Value_show(:,470));
    set(gca,'XDir','reverse')%对X方向反转
%     axis equal;
    axis([-3.5 6.5 0 20] );
    hold on

%     figure(4)
%     plot([-10:0.1:60],APF_Value_show(96,:));
% 
% %     axis equal;
%     axis([0 60 0 20] );
%     hold on
    
%     for i_Control=1:Num_Car
%         if CarPlot(i_Control).HostFlag==1
%             %处理被控车辆
%             [Carhost,Num_CoCar,CoCar,Num_obCar,obCar]=NewRelation(CarPlot,i_Control);
%             Num_obCartemp=Num_CoCar+Num_obCar;
%             obCartemp=[CoCar,obCar];
%             
%             APFPointValue=NewAPFPoint_calc(Carhost.State(1),Carhost.State(2),Carhost,Num_obCartemp,obCartemp,Num_lane,lane);
%             
%             plot3(Carhost.State(1),Carhost.State(2),APFPointValue+1.5,'r*');
%             
%             
%             axis equal;
%             hold on;
%             %axis([0 LaneLength lane(1)-5 lane(Num_lane)+5 ])    
%             if i_Control == HostID
%                 for p=1:1:Np
%                     PridictAPFtemp=Carhost.PredictState(p,6);
%                     if PridictAPFtemp>20
%                         PridictAPFtemp=20;
%                     end
%                     plot3(Carhost.PredictState(p,1),Carhost.PredictState(p,2),PridictAPFtemp+16,'g*');
%                 end
%             else
%                 for p=1:1:Np
%                     PridictAPFtemp=Carhost.PredictState(p,6);
%                     if PridictAPFtemp>20
%                         PridictAPFtemp=20;
%                     end
%                      plot3(Carhost.PredictState(p,1),Carhost.PredictState(p,2),Carhost.PredictState(p,6)+16,'b*');
%                 end
%             end
%             
%             hold on;
%             axis equal;
%             axis([LaneStart LaneEnd lane(1)-2 lane(Num_lane)+2 0 37] );
%             az = -90;
%             el = 60;
%             view(az,el);          
%         end
%     end
%     
%     pause(0.05);
%     hold off;  
