input('Hey u idiot,r u sure u wannna Begin?');
clear all;
%% ==============================================
%   ��ʼ������������ϵͳ����������״̬�����������ϰ���Ϣ����
%  =====================================================
%ϵͳ������ʼ��
N=2200;   %ϵͳ���沽��
T=0.02;   %ϵͳ��������
Tp=0.25;  %Ԥ�����Ԥ������
Np=30;
Nc=2;

%����������ʼ�����ã�������賵������ֱ�ߣ�ƽ����x�ᣩ


lane(1).id=0;   
lane(1).width = 0;
lane(1).type = 0;   % 0Ϊ��·�߽� 1 Ϊ��·�м䳵����
lane(2).id=-1;  
lane(2).width = 3.5;
lane(2).type = 1;   % 0Ϊ��·�߽� 1 Ϊ��·�м䳵����
lane(3).id=-2;  
lane(3).width = 3.5;
lane(3).type = 0;   % 0Ϊ��·�߽� 1 Ϊ��·�м䳵����
% lane(4).id=-3;  
% lane(4).width = 3.5;
% lane(4).type = 0;   % 0Ϊ��·�߽� 1 Ϊ��·�м䳵����

Road = SetRoad([-20 0 0],[200 300*pi 200 200 400],[1 3 1 3 1],[pi/2 0;600 0;0 0;-600 0;0 0],lane,0.1);

[temp_m,Num_lane]=size(lane);

%% ==============================================
%   ����״̬��ʼ������
%  =====================================================


%����״̬����

% Car(1).State=[0    -1.5   0    20     0    ];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
%ת��뾶400
Car(1).RoadLocation = [1,-1, 135, 17.2, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng��ǰ��ƫ�ǣ�]
Car(2).RoadLocation = [1,-2, 95, 18, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng��ǰ��ƫ�ǣ�]
Car(3).RoadLocation = [1,-2, 75, 21, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng��ǰ��ƫ�ǣ�]
Car(4).RoadLocation = [1,-2, 60, 21, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng��ǰ��ƫ�ǣ�]
Car(5).RoadLocation = [1,-2, 45,  21, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng��ǰ��ƫ�ǣ�]
Car(6).RoadLocation = [1,-2, 30,  21, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng��ǰ��ƫ�ǣ�]
Car(7).RoadLocation = [1,-2, 15,  21, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng��ǰ��ƫ�ǣ�]
Car(8).RoadLocation = [1,-2, 0,  21, 0 ]; %[RoadNum, LaneID, s ,v,WheelAng��ǰ��ƫ�ǣ�]
% Car(2).State=[75    -1.75    0     19     0.0];     
% Car(3).State=[50,  -1.75,   0,    15,      0.0];
% Car(4).State=[40   -5.25    0     19     0.0];     
% Car(5).State=[10,  -1.75,   0,    23,      0.0];
% Car(6).State=[0    -5.25    0     24     0.0];     

%ת��뾶400
% Car(1).State=[30,  -3.1,   0.13,    17,      0.0128];
% Car(2).State=[0    -5.25   0       20      0.0128];  

% % Car(3).State=[90      -1.5   0    20     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
% Car(3).State=[60       1.5   0    24     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
% Car(5).State=[30       1.5   0    23     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
% Car(6).State=[0        4.5   0    26     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�



[temp_m,Num_Car]=size(Car);
%��ʼ����������ΪĬ��ֵ
for temp_i=1:Num_Car
    CarState = CarLocation(Car(temp_i).RoadLocation,Road);    
    Car(temp_i).State=[CarState.x CarState.y CarState.phi CarState.v CarState.wheelang];
    Car(temp_i).Roadmilestore = Roadloc(Car(temp_i),Road );
    Car(temp_i).length=4.8;
    Car(temp_i).width=1.8;
    Car(temp_i).platoon=temp_i;
    Car(temp_i).PltnNum=1;
    Car(temp_i).ID=temp_i;
    Car(temp_i).PltnEn=0;  %ʹ���Ƿ���Լ�����������
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
%���س�������
% Car(2).HostFlag=1;
% Car(1).HostFlag=1;
Car(3).HostFlag=1;
Car(4).HostFlag=1;
Car(5).HostFlag=1;
Car(6).HostFlag=1;
Car(7).HostFlag=1;
Car(8).HostFlag=1;
% Car(6).HostFlag=1;

%��ͨ�ų�������
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

%���Ӳ�������

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


%���س�����������
% Car(4).SpeedSet=26;
% Car(5).SpeedSet=26;
% Car(6).SpeedSet=26;

% =========================================
% MPCĿ�꺯����ر�����ʼ��
% =========================================

global ymin ymax;
ymin=-3; %�߽�����Լ��
ymax=6;  %�߽�����Լ��

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
% ���������¼
% =========================================

Cartemp=cell(N,1); %��¼ÿ�����沽���еĸ�������״̬
%% ��ʼ���
for j=1:1:N
 tic   
    
    %�����HostID�����Ŀ���������
     for i_Control=1:Num_Car
        if Car(i_Control).HostFlag==1
            %�����س���
            [a_actual,deltaf_actual] = NewAPFMpc(Car,i_Control,Road,Np,Nc,Q_f,X_f,Y_f,R_f,Pa_f,Pw_f,Car(i_Control).intraDis,Tp);          
            CarNew(i_Control)=Car(i_Control);
            CarNew(i_Control) = NewCarSolve( Car(i_Control),a_actual,deltaf_actual,T);
            %���汻�س������Ʊ���������
            CarNew(i_Control).a_real=a_actual;
            CarNew(i_Control).deltaf_real=deltaf_actual;
            CarNew(i_Control).v_real=CarNew(i_Control).State(4);
            CarNew(i_Control).wheelang_real=CarNew(i_Control).State(5);
            [DisToEachLane,Roadhdg,Roadwidth,DisToCenter,RoadNum,s,LocationLaneID] = DisToLaneFunction( CarNew(i_Control).State(1),CarNew(i_Control).State(2),Road);
            CarNew(i_Control).RoadLocation = [RoadNum,LocationLaneID,s,CarNew(i_Control).State(4),CarNew(i_Control).State(5)];
            CarNew(i_Control).Roadmilestore = Roadloc(CarNew(i_Control),Road );
        else
            %�������ϰ�����
                       
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