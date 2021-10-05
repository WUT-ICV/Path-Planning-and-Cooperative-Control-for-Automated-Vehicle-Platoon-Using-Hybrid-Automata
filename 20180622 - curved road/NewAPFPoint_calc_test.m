function APF_point = NewAPFPoint_calc_test(x,y,Carlocal,Num_obCar,obCar,Num_lane,lane)
%UNTITLED Summary of this function goes here
% =========================================
%   功能：计算点(x,y)处的势场值
% =========================================
APFPoint_lane=0;
APFPoint_obCar=0;
APFPoint_road=0;
%% 计算车道线势场
k=0.8;         %车道线势场增益因子
r_eta=0.8;   %车道线势场衰减因子，值越大衰减越慢

for l_i = 1:1:Num_lane
    if abs(lane(l_i)-Carlocal.State(2))<3
        APFPoint_lane = APFPoint_lane + k*exp(-(y-lane(l_i))^2/(2*r_eta^2));
    else
        APFPoint_lane = APFPoint_lane + 2*k*exp(-(y-lane(l_i))^2/(2*r_eta^2));
    end    
end
%    APFPoint_road = 0.2/((y-3)^2+0.00001)+0.2/((y+3)^2+0.00001);
%% 计算道路边界势场
% APFPoint_road = (2*(y-(lane(1)+0.9))^2+0.5)*(y<lane(1))+(2*(y-(lane(Num_lane)-0.9))^2+0.5)*(y>lane(Num_lane));
  APFPoint_road = (10*(y-(lane(1)+0.8))^2)*(y<lane(1)+0.8)+(10*(y-(lane(Num_lane)-0.8))^2)*(y>lane(Num_lane)-0.8);

%% 计算环境车辆势场
%相对速度
v_x=Carlocal.State(4)*cos(Carlocal.State(3)*pi/180);
v_y=Carlocal.State(4)*sin(Carlocal.State(3)*pi/180);

for c_i = 1:1:Num_obCar                         %障碍车辆从car_i=1到car_i=Num_obCar
 
    if obCar(c_i).platoon~=Carlocal.platoon
        %障碍车辆参数初始化
        obx(c_i)=obCar(c_i).State(1);                  %第c_i辆障碍车辆x坐标
        oby(c_i)=obCar(c_i).State(2);                  %第c_i辆障碍车辆y坐标
        obtheta(c_i)=obCar(c_i).State(3)/180*pi;       %第c_i辆障碍车辆航向角
        SafeDis_factor=obCar(c_i).State(4)*0.08;
        len(c_i)=obCar(c_i).length*SafeDis_factor+3.001;            %第c_i辆障碍车辆长度1/2+1/2长度的安全距离
        %w(c_i)=obCar(c_i).width/2+0.001;              %第c_i辆障碍车辆宽度1/2
        w(c_i)=obCar(c_i).width/2+0.001;            %第c_i辆障碍车辆宽度1/2 20160330

        obv_x(c_i)=obCar(c_i).State(4)*cos(obCar(c_i).State(3)); 
        obv_y(c_i)=obCar(c_i).State(4)*sin(obCar(c_i).State(3)); 

        vr_x(c_i)=v_x-obv_x(c_i); %第c_i辆障碍车辆与被控车辆相对纵向速度
        vr_y(c_i)=v_y-obv_y(c_i); %第c_i辆障碍车辆与被控车辆相对横向速度

        s_x(c_i)=1*vr_x(c_i)+3;
        s_y(c_i)=1*vr_y(c_i);    


        %把计算势场点坐标（x,y）变换到障碍车辆坐标系下点（xr(c_i),yr(c_i)）
        xr(c_i) = (x-obx(c_i))*cos(obtheta(c_i))+(y-oby(c_i))*sin(obtheta(c_i));
        yr(c_i) = (y-oby(c_i))*cos(obtheta(c_i))-(x-obx(c_i))*sin(obtheta(c_i));

        %计算世界坐标系下点（x,y）相对于第c_i辆车的势场，计算时用相对于c_i辆车的相对坐标（xr(c_i),yr(c_i)）来计算
        %静态势场距离

        %APFPoint_obCar = APFPoint_obCar + 10/(xr(c_i)^2+(3*yr(c_i))^2+0.000001);

        %动态势场距离
        eta=0.55;
%          if (vr_x(c_i)>0 || abs(s_y(c_i))>=1.5)
        if (vr_x(c_i)>0)
            Acar=10;
%              Acar=Ucar*(xr(c_i)>=-(len(c_i)+s_x(c_i)))+... 
%              Ucar*exp(-(abs(xr(c_i)+(s_x(c_i)+len(c_i))))^2/((2*vr_x(c_i))^2));
% 
%             Acar=Ucar*(xr(c_i)>=-(len(c_i)+s_x(c_i)))+... 
%              vr_x(c_i)/(abs(xr(c_i)+(s_x(c_i)+len(c_i))));
%         elseif ((obx(c_i)+len(c_i)>Carlocal.State(1)))
        elseif ((obx(c_i)+len(c_i)>x))
%         elseif (abs(oby(c_i)-Carlocal.State(2))<4.5)
            Ucar=10;
             Acar=Ucar*(xr(c_i)>=-(len(c_i)+s_x(c_i)))+... 
             Ucar*exp(-(abs(xr(c_i)+(s_x(c_i)+len(c_i))))^2/(2*vr_x(c_i)^2));

            Acar=Ucar*(xr(c_i)>=-(len(c_i)+s_x(c_i)))+... 
             0/(abs(xr(c_i)+(s_x(c_i)+len(c_i))));
        else
            Ucar=0;
            Acar=0;
        end

        

        Acar(find(Acar>10))=10;

        line1(c_i)=w(c_i)/2*cos(pi/s_x(c_i)*xr(c_i)+pi+len(c_i)*pi/s_x(c_i))-w(c_i)/2;
        line2(c_i)=-w(c_i)/2*cos(pi/s_x(c_i)*xr(c_i)+pi+len(c_i)*pi/s_x(c_i))+w(c_i)/2;

        D_d(c_i)=(sqrt((xr(c_i)+(len(c_i)))^2+(yr(c_i))^2)-(w(c_i)/2))*((sqrt((xr(c_i)+(len(c_i)))^2+(yr(c_i))^2)>(w(c_i)/2)) & (xr(c_i)<-len(c_i)))+...
                sqrt((xr(c_i)-len(c_i))^2+(yr(c_i)+w(c_i)+s_y(c_i)*(s_y(c_i)>0))^2)*(xr(c_i)>len(c_i) & yr(c_i)<-w(c_i))+...
                sqrt((xr(c_i)-len(c_i))^2+(yr(c_i)-w(c_i)-s_y(c_i)*(s_y(c_i)<0))^2)*(xr(c_i)>len(c_i) & yr(c_i)>w(c_i))+...
                (xr(c_i)-len(c_i))*(xr(c_i)>len(c_i) & yr(c_i)>=-w(c_i) & yr(c_i)<=w(c_i))+...
                (-w(c_i)-yr(c_i)-s_y(c_i)*(s_y(c_i)>0))*(yr(c_i)<-w(c_i)-s_y(c_i)*(s_y(c_i)>0) & xr(c_i)>=-len(c_i) & xr(c_i)<=len(c_i))+...
                (yr(c_i)-w(c_i)+s_y(c_i)*(s_y(c_i)<0))*(yr(c_i)>w(c_i)-s_y(c_i)*(s_y(c_i)<0) & xr(c_i)>=-len(c_i) & xr(c_i)<=len(c_i));
            
%          APFPoint_obCar = APFPoint_obCar + Acar*exp(-(D_d(c_i))^2/(2*eta^2));
         APFPoint_obCar = max(APFPoint_obCar , Acar*exp(-(D_d(c_i))^2/(2*eta^2)));
    
    elseif ((obCar(c_i).platoon==Carlocal.platoon)  && (obCar(c_i).PltnNum==Carlocal.PltnNum-1))
        %障碍车辆参数初始化
        obx(c_i)=obCar(c_i).State(1);                  %第c_i辆障碍车辆x坐标
        oby(c_i)=obCar(c_i).State(2);                  %第c_i辆障碍车辆y坐标
        obtheta(c_i)=obCar(c_i).State(3)/180*pi;       %第c_i辆障碍车辆航向角
        len(c_i)=obCar(c_i).length+0.001;         %第c_i辆障碍车辆长度1/2+1/2长度的安全距离
        %w(c_i)=obCar(c_i).width/2+0.001;            %第c_i辆障碍车辆宽度1/2
        w(c_i)=obCar(c_i).width/2+0.001;            %第c_i辆障碍车辆宽度1/2 20160330

        obv_x(c_i)=obCar(c_i).State(4)*cos(obCar(c_i).State(3)); 
        obv_y(c_i)=obCar(c_i).State(4)*sin(obCar(c_i).State(3)); 

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
            Ucar=10;
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
                
    end
  
end

APF_speed=-0.01*(x-Carlocal.State(1))+2;
if APF_speed<0
    APF_speed=0;
end


% APF_point=APFPoint_lane;
% APF_point=APFPoint_lane*(abs(APFPoint_obCar)<=0.04)+APFPoint_obCar*(abs(APFPoint_obCar)>0.04)+APF_speed+APFPoint_road;
  APF_point=APFPoint_obCar*(APFPoint_obCar>APFPoint_lane)+1/2*(APFPoint_obCar+APFPoint_lane)*(APFPoint_obCar<=APFPoint_lane)+APF_speed+APFPoint_road;
% APF_point=APFPoint_obCar;
end

