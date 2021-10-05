input('Hey u idiot,r u sure u wannna Begin?');
clear all;
%% ==============================================
%   初始化参数，包含系统参数、车辆状态参数、环境障碍信息参数
%  =====================================================
%系统参数初始化
N=2200;   %系统仿真步长
T=0.02;   %系统采样周期
Tp=0.25;  %预测控制预测周期
Np=30;
Nc=2;

%车道参数初始化设置（这里假设车道线是直线，平行于x轴）


lane(1).id=0;   
lane(1).width = 0;
lane(1).type = 0;   % 0为道路边界 1 为道路中间车道线
lane(2).id=-1;  
lane(2).width = 3.5;
lane(2).type = 1;   % 0为道路边界 1 为道路中间车道线
lane(3).id=-2;  
lane(3).width = 3.5;
lane(3).type = 0;   % 0为道路边界 1 为道路中间车道线
% lane(4).id=-3;  
% lane(4).width = 3.5;
% lane(4).type = 0;   % 0为道路边界 1 为道路中间车道线

Road = SetRoad([-20 0 0],[200 300*pi 200 200 400],[1 3 1 3 1],[pi/2 0;600 0;0 0;-600 0;0 0],lane,0.1);

[temp_m,Num_lane]=size(lane);

%% ==============================================
%   车辆状态初始化参数
%  =====================================================


%车辆状态设置

% Car(1).State=[0    -1.5   0    20     0    ];     % x y phi v WheelAng（前轮偏角）
%转弯半径400
Car(1).RoadLocation = [1,-1, 135, 17.2, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng（前轮偏角）]
Car(2).RoadLocation = [1,-2, 95, 18, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng（前轮偏角）]
Car(3).RoadLocation = [1,-2, 75, 21, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng（前轮偏角）]
Car(4).RoadLocation = [1,-2, 60, 21, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng（前轮偏角）]
Car(5).RoadLocation = [1,-2, 45,  21, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng（前轮偏角）]
Car(6).RoadLocation = [1,-2, 30,  21, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng（前轮偏角）]
Car(7).RoadLocation = [1,-2, 15,  21, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng（前轮偏角）]
Car(8).RoadLocation = [1,-2, 0,  21, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng（前轮偏角）]
% Car(2).State=[75    -1.75    0     19     0.0];     
% Car(3).State=[50,  -1.75,   0,    15,      0.0];
% Car(4).State=[40   -5.25    0     19     0.0];     
% Car(5).State=[10,  -1.75,   0,    23,      0.0];
% Car(6).State=[0    -5.25    0     24     0.0];     

%转弯半径400
% Car(1).State=[30,  -3.1,   0.13,    17,      0.0128];
% Car(2).State=[0    -5.25   0       20      0.0128];  

% % Car(3).State=[90      -1.5   0    20     0];     % x y phi v WheelAng（前轮偏角）
% Car(3).State=[60       1.5   0    24     0];     % x y phi v WheelAng（前轮偏角）
% Car(5).State=[30       1.5   0    23     0];     % x y phi v WheelAng（前轮偏角）
% Car(6).State=[0        4.5   0    26     0];     % x y phi v WheelAng（前轮偏角）



[temp_m,Num_Car]=size(Car);
%初始化车辆参数为默认值
for temp_i=1:Num_Car
    CarState = CarLocation(Car(temp_i).RoadLocation,Road);    
    Car(temp_i).State=[CarState.x CarState.y CarState.phi CarState.v CarState.wheelang];
    Car(temp_i).Roadmilestore = Roadloc(Car(temp_i),Road );
    Car(temp_i).length=4.8;
    Car(temp_i).width=1.8;
    Car(temp_i).platoon=temp_i;
    Car(temp_i).PltnNum=1;
    Car(temp_i).ID=temp_i;
    Car(temp_i).PltnEn=0;  %使能是否可以加入其它车队
    Car(temp_i).LeaderID=temp_i;
    Car(temp_i).PltnLength=1;
    Car(temp_i).intraDis=15;
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
%被控车辆设置
% Car(2).HostFlag=1;
% Car(1).HostFlag=1;
Car(3).HostFlag=1;
Car(4).HostFlag=1;
Car(5).HostFlag=1;
Car(6).HostFlag=1;
Car(7).HostFlag=1;
Car(8).HostFlag=1;
% Car(6).HostFlag=1;

%可通信车辆设置
% Car(1).comEn=1;
% Car(1).comDelay=0.02;
% Car(2).comEn=1;
% Car(2).comDelay=0.02;
Car(3).comEn=2;
Car(3).comDelay=0.02;
Car(4).comEn=2;
Car(4).comDelay=0.02;
Car(5).comEn=2;
Car(5).comDelay=0.02;
Car(6).comEn=2;
Car(6).comDelay=0.02;
Car(7).comEn=2;
Car(7).comDelay=0.02;
Car(8).comEn=2;
Car(8).comDelay=0.02;
% Car(5).comEn=1;
% Car(5).comDelay=0.02;
% Car(6).comEn=1;
% Car(6).comDelay=0.02;

%车队参数设置

% Car(1).PltnEn=1;
% Car(2).PltnEn=1;
Car(3).PltnEn=1;
Car(4).PltnEn=1;
Car(5).PltnEn=1;
Car(6).PltnEn=1;
Car(7).PltnEn=1;
Car(8).PltnEn=1;

% Car(4).platoon=4;
% Car(4).PltnNum=1;
% Car(5).platoon=5;
% Car(5).PltnNum=1;
% Car(6).platoon=6;
% Car(6).PltnNum=1;


%被控车辆限速设置
% Car(4).SpeedSet=26;
% Car(5).SpeedSet=26;
% Car(6).SpeedSet=26;

% =========================================
% MPC目标函数相关变量初始化
% =========================================

global ymin ymax;
ymin=-3; %边界下限约束
ymax=6;  %边界上限约束

global X_predict Y_predict PHI_predict v_predict WheelAng_predict APF_Value X_real Y_real v_real ;

global aTestCounter_DisToLaneFunction;
aTestCounter_DisToLaneFunction=0;

X_predict = zeros(Np,1);
Y_predict = zeros(Np,1);
PHI_predict = zeros(Np,1);
v_predict = zeros(Np,1);
WheelAng_predict = zeros(Np,1);

APF_Value=zeros(Np+1,1);

Q_f=50;       % APF factor
X_f=20;
Y_f=15;
R_f=5;       % speed factor
Pa_f=60;      % accelerator factor
Pw_f=60;      % wheelangacceleratoror factor

    for init_i=1:1:Num_Car    
        Car(init_i).PredictRoadLocation(1,1:5) = Car(init_i).RoadLocation(1,1:5);
        Car(init_i).PredictState(1,1:5) = Car(init_i).State;
        Car(init_i).PredictRoadmilestore(1) = Roadloc(Car(init_i),Road );
        for i=1:Np-1
            Car(init_i).PredictState(i+1,:) = Car(init_i).PredictState(i,1:5);
            Car(init_i).PredictRoadLocation(i+1,:) = Car(init_i).PredictRoadLocation(i,1:5);           
            s_temp = Car(init_i).PredictRoadLocation(i,3) + Tp * Car(init_i).PredictRoadLocation(i,4);             
            if s_temp > Road(Car(init_i).PredictRoadLocation(i,1)).geometry.length
                Car(init_i).PredictRoadLocation(i+1,3) = s_temp - Road(Car(init_i).PredictRoadLocation(i,1)).geometry.length;
                Car(init_i).PredictRoadLocation(i+1,1) = Car(init_i).PredictRoadLocation(i,1)+1;                
            else
                Car(init_i).PredictRoadLocation(i+1,3) = s_temp;
            end
            CarState = CarLocation(Car(init_i).PredictRoadLocation(i+1,:),Road);    
            Car(init_i).PredictState(i+1,1:5)=[CarState.x CarState.y CarState.phi CarState.v CarState.wheelang];
            TempCar.RoadLocation = Car(init_i).PredictRoadLocation(i+1,1:5);
            Car(init_i).PredictRoadmilestore(i+1,:) = Roadloc(TempCar,Road);

        end          
    end

% =========================================
% 仿真参数记录
% =========================================

Cartemp=cell(N,1); %记录每个仿真步长中的各个车辆状态
%% 开始求解
for j=1:1:N
 tic   
    
    %计算第HostID辆车的控制输入量
     for i_Control=1:Num_Car
        if Car(i_Control).HostFlag==1
            %处理被控车辆
            [a_actual,deltaf_actual] = NewAPFMpc(Car,i_Control,Road,Np,Nc,Q_f,X_f,Y_f,R_f,Pa_f,Pw_f,Car(i_Control).intraDis,Tp);          
            CarNew(i_Control)=Car(i_Control);
            CarNew(i_Control) = NewCarSolve( Car(i_Control),a_actual,deltaf_actual,T);
            %保存被控车辆控制变量及参数
            CarNew(i_Control).a_real=a_actual;
            CarNew(i_Control).deltaf_real=deltaf_actual;
            CarNew(i_Control).v_real=CarNew(i_Control).State(4);
            CarNew(i_Control).wheelang_real=CarNew(i_Control).State(5);
            [DisToEachLane,Roadhdg,Roadwidth,DisToCenter,RoadNum,s,LocationLaneID] = DisToLaneFunction( CarNew(i_Control).State(1),CarNew(i_Control).State(2),Road);
            CarNew(i_Control).RoadLocation = [RoadNum,LocationLaneID,s,CarNew(i_Control).State(4),CarNew(i_Control).State(5)];
            CarNew(i_Control).Roadmilestore = Roadloc(CarNew(i_Control),Road );
        else
            %处理环境障碍车辆
                       
            CarNew(i_Control)=Car(i_Control);           
            s_temp = CarNew(i_Control).RoadLocation(3) + T * CarNew(i_Control).RoadLocation(4);
            if s_temp > Road(CarNew(i_Control).RoadLocation(1)).geometry.length
                CarNew(i_Control).RoadLocation(3) = s_temp - Road(CarNew(i_Control).RoadLocation(1)).geometry.length;
                CarNew(i_Control).RoadLocation(1) = CarNew(i_Control).RoadLocation(1)+1;                
            else
                CarNew(i_Control).RoadLocation(3) = s_temp;
            end
            CarState = CarLocation(CarNew(i_Control).RoadLocation,Road);    
            CarNew(i_Control).State=[CarState.x CarState.y CarState.phi CarState.v CarState.wheelang];
            CarNew(i_Control).Roadmilestore = Roadloc(Car(i_Control),Road );
        end
    end
    
  
    Cartemp{j,1} = CarNew;
    Car=CarNew;
    
    for i_Control=1:Num_Car
        if Car(i_Control).HostFlag==1            
            Car=CarStrategy(Car,i_Control);                            % 20160914
        end
    end
    
    fprintf('Update Process, j=%4.2f\n',j)
    for i_Control=1:Num_Car
        if Car(i_Control).HostFlag==1
            fprintf('Update Process, HostCar_Num=%4.2f',i_Control)
%             fprintf('    x=%4.2f',Car(i_Control).State(1))
%             fprintf('    y=%4.2f',Car(i_Control).State(2))
            fprintf('    laneID=%4.2f',Car(i_Control).RoadLocation(2))
            fprintf('    s=%4.2f',Car(i_Control).RoadLocation(3))
            fprintf('    v=%4.2f',Car(i_Control).State(4))
%             fprintf('Update Process, HostCar_y=%4.2f',Car(i_Control).State(2))
%             fprintf('Update Process, HostCar_v=%4.2f\n',Car(i_Control).State(4))
            fprintf('    Max_APFValue=%6.2f\n',max(Car(i_Control).PredictState(:,6)))            
        end
    end
    for i_Control=1:Num_Car
        if Car(i_Control).HostFlag==1
            fprintf('Update Process, HostCar_platoon=%4.2f',Car(i_Control).platoon)
            fprintf('    PltnLength=%4.2f',Car(i_Control).PltnLength)
            fprintf('    PltnNum=%4.2f\n',Car(i_Control).PltnNum)
        end
    end
   
toc

end