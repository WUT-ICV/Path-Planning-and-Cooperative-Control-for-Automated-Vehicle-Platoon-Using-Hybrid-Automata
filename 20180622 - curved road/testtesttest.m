



lane(1).id=0;   
lane(1).width = 0;
lane(1).type = 0;   % 0为道路边界 1 为道路中间车道线
lane(2).id=-1;  
lane(2).width = 3.5;
lane(2).type = 1;   % 0为道路边界 1 为道路中间车道线
lane(3).id=-2;  
lane(3).width = 3.5;
lane(3).type = 1;   % 0为道路边界 1 为道路中间车道线
lane(4).id=-3;  
lane(4).width = 3.5;
lane(4).type = 0;   % 0为道路边界 1 为道路中间车道线

Road = SetRoad([-100 0 0],[200 100*pi 100*pi 100],[1 3 3 1],[0 0;200 0;-200 0;0 0],lane,0.1);

% Road = SetRoad([0 0 0],[100 100*pi 100],[1 3 1],[0 0;200 0;0 0]);
testpoint=[305,205];


% Car(1).State=[100,   -5.25,   0,    15,      0.0];
Car(1).RoadLocation = [1,-2,190,0,0]; % RoadNum, LaneID, s
[temp_m,Num_Car]=size(Car);
for temp_i=1:Num_Car
    CarState = CarLocation(Car(temp_i).RoadLocation,Road);
    Car(temp_i).State=[CarState.x CarState.y CarState.phi CarState.v CarState.wheelang];
    Car(temp_i).length=4.8;
    Car(temp_i).width=1.8;
    Car(temp_i).platoon=temp_i;
    Car(temp_i).PltnNum=1;
    Car(temp_i).ID=temp_i;
    Car(temp_i).PltnEn=0;  %使能是否可以加入其它车队
    Car(temp_i).LeaderID=temp_i;
    Car(temp_i).PltnLength=1;
    Car(temp_i).intraDis=10;
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
    Car(temp_i).SetlaneEn=0;
    Car(temp_i).Setlane=1.5;       
end
% CarState = CarLocation(Car(1).RoadLocation,Road);
% [dis,Roadhdg,Roadwidth,DisToCenter,RoadNum,s,locatoinlane]=DisToLaneFunction(CarState.x,CarState.y,Road);
tic
for i=1:100000
[dis1,Roadhdg1,Roadwidth1,DisToCenter1,RoadNum1,s1,locationlane1]=NewDisToLaneFunction(Car(1).State(1),Car(1).State(2),Road,Car(1));
end
toc
tic
for i=1:100000
[dis,Roadhdg,Roadwidth,DisToCenter,RoadNum,s,locationlane]=DisToLaneFunction(Car(1).State(1),Car(1).State(2),Road);
end
toc
tic
for i=1:100000
[dis2,Roadhdg2,Roadwidth2,DisToCenter2,RoadNum2,s2,locationlane2]=testDisToLaneFunction(Car(1).State(1),Car(1).State(2),Road);
end
toc

plot(Car(1).State(1),Car(1).State(2),'b*');
hold on


% Road = SetRoad([-20 0 pi/6],[150 300*pi 200 200 400],[1 3 1 3 1],[pi/2 0;600 0;0 0;-600 0;0 0],lane,0.2);
[temp_m,Num_Section] = size(Road);

for i= 1 : Num_Section
    for lane_i = 1:length(Road(i).lane)
        if Road(i).lane(lane_i).type == 0
            plot(Road(i).lane(lane_i).sample_x,Road(i).lane(lane_i).sample_y,'r-' );  
            hold on
            plot(Road(i).samplepoint.x,Road(i).samplepoint.y,'b-' );      
            hold on
        else
            plot(Road(i).lane(lane_i).sample_x,Road(i).lane(lane_i).sample_y,'r--' );
            hold on
        end      

    end
end
axis equal
hold off

% for i= 1 : Num_Section
%     if Road(i).geometry.type == 1 
%         StartPoint = [Road(i).geometry.SPoint(1) Road(i).geometry.SPoint(2)];
%         EndPoint = [Road(i).geometry.SPoint(1)+ Road(i).geometry.length * cos(Road(i).geometry.hdg) 
%                      Road(i).geometry.SPoint(2)+ Road(i).geometry.length * sin(Road(i).geometry.hdg)];
%         lane(1).StartPoint = StartPoint;
%         lane(1).EndPoint = EndPoint;
%         for lane_i=2:1:length(Road(i).lane)
%             width=Road(i).lane(lane_i).width;
%             lane(lane_i).StartPoint = [lane(lane_i-1).StartPoint(1)+ width*cos(Road(i).geometry.hdg-pi/2) lane(lane_i-1).StartPoint(2)+ width*sin(Road(i).geometry.hdg-pi/2)];
%             lane(lane_i).EndPoint = [lane(lane_i-1).EndPoint(1)+ width*cos(Road(i).geometry.hdg-pi/2) lane(lane_i-1).EndPoint(2)+ width*sin(Road(i).geometry.hdg-pi/2)];            
%         end
%         for lane_i=1:length(Road(i).lane)
%             if Road(i).lane(lane_i).type == 0
%                 plot([lane(lane_i).StartPoint(1) lane(lane_i).EndPoint(1)],[lane(lane_i).StartPoint(2) lane(lane_i).EndPoint(2)],'r-');                
%                 hold on
%             else
%                 plot([lane(lane_i).StartPoint(1) lane(lane_i).EndPoint(1)],[lane(lane_i).StartPoint(2) lane(lane_i).EndPoint(2)],'r--');
%                 hold on
%             end
%         end
%     elseif Road(i).geometry.type == 3
%         StartPoint = [Road(i).geometry.SPoint(1) Road(i).geometry.SPoint(2)];
%         if Road(i).geometry.Curve(1)>=0
%             StartHdg = Road(i).geometry.hdg - pi/2;
%             EndHdg = (Road(i).geometry.length  / ( Road(i).geometry.Curve(1) * 2))*2 +  Road(i).geometry.hdg - pi/2;
%         else
%             EndHdg = Road(i).geometry.hdg + pi/2;
%             StartHdg = (Road(i).geometry.length  / ( Road(i).geometry.Curve(1) * 2))*2 +  Road(i).geometry.hdg + pi/2;
%         end
%         
%         r = Road(i).geometry.Curve(1);
%         circPoint = [StartPoint(1)-r*sin(Road(i).geometry.hdg) StartPoint(2)+r*cos(Road(i).geometry.hdg) ];
%         t_circ = StartHdg : pi/500 : EndHdg;
%         for lane_i=1:length(Road(i).lane)
%             if Road(i).lane(lane_i).id == 0
%                 Curve_r(lane_i) = abs(Road(i).geometry.Curve(1));
%             else
%                 if Road(i).geometry.Curve(1)>=0
%                     Curve_r(lane_i) = Curve_r(lane_i-1)+ Road(i).lane(lane_i).width;
%                 else
%                     Curve_r(lane_i) = Curve_r(lane_i-1)- Road(i).lane(lane_i).width;
%                 end
%             end
%             x = Curve_r(lane_i) * cos(t_circ)+circPoint(1);
%             y = Curve_r(lane_i) * sin(t_circ)+circPoint(2);
%             if Road(i).lane(lane_i).type == 0
%                 plot(x,y,'r-');                
%                 hold on
%             else
%                 plot(x,y,'r--');
%                 hold on
%             end
%        
%         end
%         
%     end
%     axis equal;
% end


