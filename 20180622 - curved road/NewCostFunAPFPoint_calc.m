function APF_point = NewCostFunAPFPoint_calc(x,y,phi,v,Carlocal,Num_obCar,obCar,obCarnow,Road)
%UNTITLED Summary of this function goes here
% =========================================
%   功能：计算点(x,y)处的势场值
% =========================================

APFPoint_lane=0;
APFPoint_obCar=0;
APFPoint_road=0;
%% 计算车道线势场
k=1;         %车道线势场增益因子
r_eta=0.6;   %车道线势场衰减因子，值越大衰减越慢
offset = 1; %道路势场偏移量

% [DisToLane,road_hdg,Roadwidth,DisToRoadCenter,RoadNum,s,locationlaneID]=testDisToLaneFunction(x,y,Road);

[DisToLane,road_hdg,Roadwidth,DisToRoadCenter,RoadNum,s,locationlaneID]=DisToLaneFunction(x,y,Road);

for lane_i = 1:length(Road(RoadNum).lane)    
    if DisToLane(lane_i) < Road(RoadNum).lane(2).width
        APFPoint_lane = APFPoint_lane + k*exp(-(DisToLane(lane_i))^2/(2*r_eta^2));
    else
        APFPoint_lane = APFPoint_lane + 2*k*exp(-(DisToLane(lane_i))^2/(2*r_eta^2));
    end   
end
% APFPoint_road = (5 * (DisToRoadCenter - (Roadwidth/2 - offset))) * ((DisToRoadCenter > (Roadwidth/2 - offset)));
APFPoint_road = 20 * (DisToRoadCenter>Roadwidth/2) + 20*exp(-(DisToLane(1))^2/(2*0.5^2))*(DisToRoadCenter<=Roadwidth/2)+...
                                                     20*exp(-(DisToLane(lane_i))^2/(2*0.5^2))*(DisToRoadCenter<=Roadwidth/2);
%% 计算环境车辆势场
%相对速度
v_x=v*cos(phi*pi/180);
v_y=v*sin(phi*pi/180);
v_x=Carlocal.State(4)*cos(Carlocal.State(3)-road_hdg);  %test
v_y=Carlocal.State(4)*sin(Carlocal.State(3)-road_hdg);  %test

for c_i = 1:1:Num_obCar                         %障碍车辆从car_i=1到car_i=Num_obCar
 
%     if obCar(c_i).platoon~=Carlocal.platoon ...
%        && ~((abs(obCarnow(c_i).State(2)-Carlocal.State(2))>2  && abs(max(obCarnow(c_i).PredictState(:,2))-min(obCarnow(c_i).PredictState(:,2)) )>2.8 && obCarnow(c_i).State(1) < Carlocal.State(1) + 8  )...
%             ||( abs(max(Carlocal.PredictState(:,2))-min(Carlocal.PredictState(:,2)) )<2.8 && obCarnow(c_i).State(1) < Carlocal.State(1)))
    if obCar(c_i).platoon~=Carlocal.platoon ...
       && (obCarnow(c_i).RoadLocation(3) > Carlocal.RoadLocation(3) || obCarnow(c_i).RoadLocation(1) > Carlocal.RoadLocation(1))

        if Carlocal.ID == 1
            Carlocal.ID = 1;
        end
        
        %障碍车辆参数初始化
        obx(c_i)=obCar(c_i).State(1);                  %第c_i辆障碍车辆x坐标
        oby(c_i)=obCar(c_i).State(2);                  %第c_i辆障碍车辆y坐标
        obtheta(c_i)=obCar(c_i).State(3);              %第c_i辆障碍车辆航向角
        SafeDis_factor=obCar(c_i).State(4)*0.08;
        len(c_i)=obCar(c_i).length*SafeDis_factor+3.001;            %第c_i辆障碍车辆长度1/2+1/2长度的安全距离
        %w(c_i)=obCar(c_i).width/2+0.001;              %第c_i辆障碍车辆宽度1/2
        w(c_i)=obCar(c_i).width/2+0.001;            %第c_i辆障碍车辆宽度1/2 20160330

        obv_x(c_i)=obCar(c_i).State(4)*cos(obCar(c_i).State(3)); 
        obv_y(c_i)=obCar(c_i).State(4)*sin(obCar(c_i).State(3)); 
        obv_x(c_i)=obCar(c_i).State(4)*cos(obCar(c_i).State(3)-road_hdg); %test
        obv_y(c_i)=obCar(c_i).State(4)*sin(obCar(c_i).State(3)-road_hdg); %test

        vr_x(c_i)=v_x-obv_x(c_i); %第c_i辆障碍车辆与被控车辆相对纵向速度
        vr_y(c_i)=v_y-obv_y(c_i); %第c_i辆障碍车辆与被控车辆相对横向速度

        s_x(c_i)=1*vr_x(c_i)+3;
        s_x(c_i)=0.5*vr_x(c_i);
        
        
        s_y(c_i)=1*vr_y(c_i);    
        s_y(c_i)=0;   


        %把计算势场点坐标（x,y）变换到障碍车辆坐标系下点（xr(c_i),yr(c_i)）
        xr(c_i) = (x-obx(c_i))*cos(obtheta(c_i))+(y-oby(c_i))*sin(obtheta(c_i));
        yr(c_i) = (y-oby(c_i))*cos(obtheta(c_i))-(x-obx(c_i))*sin(obtheta(c_i));

        %计算世界坐标系下点（x,y）相对于第c_i辆车的势场，计算时用相对于c_i辆车的相对坐标（xr(c_i),yr(c_i)）来计算
        %静态势场距离

        %APFPoint_obCar = APFPoint_obCar + 10/(xr(c_i)^2+(3*yr(c_i))^2+0.000001);

        %动态势场距离
        eta=0.53;
%         if ((vr_x(c_i)>0 && (obx(c_i)>Carlocal.State(1)))|| abs(vr_y(c_i))>abs(vr_x(c_i)))
%         if (vr_x(c_i)>0 ||(obx(c_i)+len(c_i)>x) &&abs(oby(c_i)-Carlocal.State(2))<4.5))
        if (vr_x(c_i)>0)  && (xr(c_i)+len(c_i)>0) 
                Ucar=15;
             Acar=Ucar*(xr(c_i)>=-(len(c_i)+s_x(c_i)))+... 
             Ucar*exp(-(abs(xr(c_i)+(s_x(c_i)+len(c_i))))^2/(2*vr_x(c_i)^2));

            Acar=Ucar*(xr(c_i)>=-(len(c_i)+s_x(c_i)))+... 
             vr_x(c_i)/(abs(xr(c_i)+(s_x(c_i)+len(c_i))));
%          Acar=Ucar*(xr(c_i)>=-(len(c_i)))+... 
%              Ucar*exp(-(abs(xr(c_i)+(len(c_i))))^2/((2*vr_x(c_i))^2));
%          elseif ((obx(c_i)+len(c_i)>Carlocal.State(1)))
         elseif (xr(c_i)+len(c_i)>0) %  && obCar(c_i).HostFlag~=1
%         elseif (abs(oby(c_i)-Carlocal.State(2))<4.5)
            Ucar=15;
             Acar=Ucar*(xr(c_i)>=-(len(c_i)+s_x(c_i)))+... 
             Ucar*exp(-(abs(xr(c_i)+(s_x(c_i)+len(c_i))))^2/(2*vr_x(c_i)^2));

            Acar=Ucar*(xr(c_i)>=-(len(c_i)+s_x(c_i)))+... 
             0/(abs(xr(c_i)+(s_x(c_i)+len(c_i))));
%         elseif obCar(c_i).HostFlag==1 && (abs(oby(c_i)-Carlocal.State(2))>2) && (obx(c_i)+len(c_i)>Carlocal.State(1)) && obCar(c_i).State(4)>Carlocal.State(4)+5;
%             Ucar=15;
%              Acar=Ucar*(xr(c_i)>=-(len(c_i)+s_x(c_i)))+... 
%              Ucar*exp(-(abs(xr(c_i)+(s_x(c_i)+len(c_i))))^2/(2*vr_x(c_i)^2));
% 
%             Acar=Ucar*(xr(c_i)>=-(len(c_i)+s_x(c_i)))+... 
%              0/(abs(xr(c_i)+(s_x(c_i)+len(c_i))));
        else
            Ucar=0;
            Acar=0;
        end

        

        Acar(find(Acar>10))=10;

        line1(c_i)=w(c_i)/2*cos(pi/s_x(c_i)*xr(c_i)+pi+len(c_i)*pi/s_x(c_i))-w(c_i)/2;
        line2(c_i)=-w(c_i)/2*cos(pi/s_x(c_i)*xr(c_i)+pi+len(c_i)*pi/s_x(c_i))+w(c_i)/2;

        D_d(c_i)=abs(yr(c_i))*(xr(c_i)<-(len(c_i)+s_x(c_i)))+...
                (line1(c_i)-yr(c_i))*(xr(c_i)<-len(c_i) & xr(c_i)>=-(len(c_i)+s_x(c_i)) & yr(c_i)<line1(c_i))+...
                (yr(c_i)-line2(c_i))*(xr(c_i)<-len(c_i) & xr(c_i)>=-(len(c_i)+s_x(c_i)) & yr(c_i)>line2(c_i))+...
                sqrt((xr(c_i)-len(c_i))^2+(yr(c_i)+w(c_i)+s_y(c_i)*(s_y(c_i)>0))^2)*(xr(c_i)>len(c_i) & yr(c_i)<-w(c_i))+...
                sqrt((xr(c_i)-len(c_i))^2+(yr(c_i)-w(c_i)-s_y(c_i)*(s_y(c_i)<0))^2)*(xr(c_i)>len(c_i) & yr(c_i)>w(c_i))+...
                (xr(c_i)-len(c_i))*(xr(c_i)>len(c_i) & yr(c_i)>=-w(c_i) & yr(c_i)<=w(c_i))+...
                (-w(c_i)-yr(c_i)-s_y(c_i)*(s_y(c_i)>0))*(yr(c_i)<-w(c_i)-s_y(c_i)*(s_y(c_i)>0) & xr(c_i)>=-len(c_i) & xr(c_i)<=len(c_i))+...
                (yr(c_i)-w(c_i)+s_y(c_i)*(s_y(c_i)<0))*(yr(c_i)>w(c_i)-s_y(c_i)*(s_y(c_i)<0) & xr(c_i)>=-len(c_i) & xr(c_i)<=len(c_i));
            
%          APFPoint_obCar = APFPoint_obCar + Acar*exp(-(D_d(c_i))^2/(2*eta^2));
           APFPoint_obCar = max(APFPoint_obCar , Acar*exp(-(D_d(c_i))^2/(2*eta^2)));
           
    elseif (obCar(c_i).platoon==Carlocal.platoon)  
        if (obCar(c_i).PltnNum == Carlocal.PltnNum-1)
            %障碍车辆参数初始化
            obx(c_i)=obCar(c_i).State(1);                  %第c_i辆障碍车辆x坐标
            oby(c_i)=obCar(c_i).State(2);                  %第c_i辆障碍车辆y坐标
            obtheta(c_i)=obCar(c_i).State(3);       %第c_i辆障碍车辆航向角
            len(c_i)=obCar(c_i).length+0.001;         %第c_i辆障碍车辆长度1/2+1/2长度的安全距离
            %w(c_i)=obCar(c_i).width/2+0.001;            %第c_i辆障碍车辆宽度1/2
            w(c_i)=obCar(c_i).width/2+0.001;            %第c_i辆障碍车辆宽度1/2 20160330

            obv_x(c_i)=obCar(c_i).State(4)*cos(obCar(c_i).State(3)); 
            obv_y(c_i)=obCar(c_i).State(4)*sin(obCar(c_i).State(3)); 
            obv_x(c_i)=obCar(c_i).State(4)*cos(obCar(c_i).State(3)-road_hdg); %test
            obv_y(c_i)=obCar(c_i).State(4)*sin(obCar(c_i).State(3)-road_hdg); %test

            vr_x(c_i)=v_x-obv_x(c_i); %第c_i辆障碍车辆与被控车辆相对纵向速度
            vr_y(c_i)=v_y-obv_y(c_i); %第c_i辆障碍车辆与被控车辆相对横向速度

           % s_x(c_i)=1*vr_x(c_i)+1;
            s_x(c_i)=0.001;
            s_y(c_i)=1*vr_y(c_i);    


            %把计算势场点坐标（x,y）变换到障碍车辆坐标系下点（xr(c_i),yr(c_i)）
            xr(c_i) = (x-obx(c_i))*cos(obtheta(c_i))+(y-oby(c_i))*sin(obtheta(c_i));
            yr(c_i) = (y-oby(c_i))*cos(obtheta(c_i))-(x-obx(c_i))*sin(obtheta(c_i));

            %计算世界坐标系下点（x,y）相对于第c_i辆车的势场，计算时用相对于c_i辆车的相对坐标（xr(c_i),yr(c_i)）来计算
            %静态势场距离

            %APFPoint_obCar = APFPoint_obCar + 10/(xr(c_i)^2+(3*yr(c_i))^2+0.000001);

            %动态势场距离
            eta=0.4;
            if vr_x(c_i)>0
                Ucar=15;
                 Acar=Ucar*(xr(c_i)>=-(len(c_i)+s_x(c_i)))+... 
                 Ucar*exp(-(abs(xr(c_i)+(s_x(c_i)+len(c_i))))^2/(2*3.6*vr_x(c_i)^2));

                Acar=Ucar*(xr(c_i)>=-(len(c_i)+s_x(c_i)))+... 
                 vr_x(c_i)/(abs(xr(c_i)+(s_x(c_i)+len(c_i))));
            else
                Ucar=0;
                Acar=0;
            end



            Acar(find(Acar>10))=10;

            line1(c_i)=w(c_i)/2*cos(pi/s_x(c_i)*xr(c_i)+pi+len(c_i)*pi/s_x(c_i))-w(c_i)/2;
            line2(c_i)=-w(c_i)/2*cos(pi/s_x(c_i)*xr(c_i)+pi+len(c_i)*pi/s_x(c_i))+w(c_i)/2;

            D_d(c_i)=abs(yr(c_i))*(xr(c_i)<-(len(c_i)+s_x(c_i)))+...
                    (line1(c_i)-yr(c_i))*(xr(c_i)<-len(c_i) & xr(c_i)>=-(len(c_i)+s_x(c_i)) & yr(c_i)<line1(c_i))+...
                    (yr(c_i)-line2(c_i))*(xr(c_i)<-len(c_i) & xr(c_i)>=-(len(c_i)+s_x(c_i)) & yr(c_i)>line2(c_i))+...
                    sqrt((xr(c_i)-len(c_i))^2+(yr(c_i)+w(c_i)+s_y(c_i)*(s_y(c_i)>0))^2)*(xr(c_i)>len(c_i) & yr(c_i)<-w(c_i))+...
                    sqrt((xr(c_i)-len(c_i))^2+(yr(c_i)-w(c_i)-s_y(c_i)*(s_y(c_i)<0))^2)*(xr(c_i)>len(c_i) & yr(c_i)>w(c_i))+...
                    (xr(c_i)-len(c_i))*(xr(c_i)>len(c_i) & yr(c_i)>=-w(c_i) & yr(c_i)<=w(c_i))+...
                    (-w(c_i)-yr(c_i)-s_y(c_i)*(s_y(c_i)>0))*(yr(c_i)<-w(c_i)-s_y(c_i)*(s_y(c_i)>0) & xr(c_i)>=-len(c_i) & xr(c_i)<=len(c_i))+...
                    (yr(c_i)-w(c_i)+s_y(c_i)*(s_y(c_i)<0))*(yr(c_i)>w(c_i)-s_y(c_i)*(s_y(c_i)<0) & xr(c_i)>=-len(c_i) & xr(c_i)<=len(c_i));

            APFPoint_obCar = APFPoint_obCar + Acar*exp(-(D_d(c_i))^2/(2*eta^2));
        else
            APFPoint_obCar = APFPoint_obCar;
        end
                
    end
  
end

APF_speed=-0.01*(x-Carlocal.State(1))+2;
if APF_speed<0
    APF_speed=0;
end
APF_speed = 0;

% APF_point=APFPoint_obCar;
% APF_point=APFPoint_lane*(abs(APFPoint_obCar)<=0.04)+APFPoint_obCar*(abs(APFPoint_obCar)>0.04)+APF_speed+APFPoint_road;
 APF_point=APFPoint_obCar*(APFPoint_obCar>APFPoint_lane)+1/2*(APFPoint_obCar+APFPoint_lane)*(APFPoint_obCar<=APFPoint_lane)+APF_speed+APFPoint_road;
% APF_point=APFPoint_obCar;

end

