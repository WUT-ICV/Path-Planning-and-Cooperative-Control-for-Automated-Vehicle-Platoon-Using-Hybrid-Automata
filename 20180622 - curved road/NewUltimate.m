input('Hey u idiot,r u sure u wannna Begin?');
clear all;
%% ==============================================
%   初始化参数，包含系统参数、车辆状态参数、环境障碍信息参数
%  =====================================================
%系统参数初始化
N=2000;   %系统仿真步长
T=0.02;   %系统采样周期
Tp=0.2;  %预测控制预测周期
Np=45;
Nc=2;

%车道参数初始化设置（这里假设车道线是直线，平行于x轴）
Num_lane=4;   %车道线数量
lane=zeros(Num_lane,1);
lane(1)=-3;   %车道线纵坐标
lane(2)=0;    %车道线纵坐标
lane(3)=3;    %车道线纵坐标
lane(4)=6;    %车道线纵坐标

%% ==============================================
%   车辆状态初始化参数
%  =====================================================


%车辆状态设置
Car(5).State=[ 0       1.5   0   25       0];     % x y phi v WheelAng（前轮偏角）
Car(4).State=[12      -1.5   0   25       0];     % x y phi v WheelAng（前轮偏角）
Car(3).State=[24       1.5   0   25       0];     % x y phi v WheelAng（前轮偏角）
Car(2).State=[36      -1.5   0   25       0];     % x y phi v WheelAng（前轮偏角）
Car(1).State=[48       1.5   0   25       0];     % x y phi v WheelAng（前轮偏角）
Car(6).State=[100      4.5   0   22       0];     % x y phi v WheelAng（前轮偏角）
Car(7).State=[100      1.5   0   20       0];     % x y phi v WheelAng（前轮偏角）
Car(8).State=[150     -1.5   0   20       0];     % x y phi v WheelAng（前轮偏角）

[temp_m,Num_Car]=size(Car);
%初始化车辆参数为默认值
for temp_i=1:Num_Car
    Car(temp_i).length=4.8;
    Car(temp_i).width=1.8;
    Car(temp_i).platoon=temp_i;
    Car(temp_i).PltnNum=[];
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

%被控车辆设置
Car(1).HostFlag=1;
Car(2).HostFlag=1;
Car(3).HostFlag=1;
Car(4).HostFlag=1;
Car(5).HostFlag=1;
%可通信车辆设置

Car(1).comEn=1;
Car(1).comDelay=0.02;
Car(2).comEn=1;
Car(2).comDelay=0.02;
Car(3).comEn=1;
Car(3).comDelay=0.02;
Car(4).comEn=1;
Car(4).comDelay=0.02;
Car(5).comEn=1;
Car(5).comDelay=0.02;
%车队参数设置
%{
Car(1).platoon=1;
Car(1).PltnNum=5;
Car(2).platoon=2;
Car(2).PltnNum=4;
Car(3).platoon=3;
Car(3).PltnNum=3;
Car(4).platoon=4;
Car(4).PltnNum=2;
Car(5).platoon=5;
Car(5).PltnNum=1;
%}
%被控车辆限速设置
Car(1).SpeedSet=25;
Car(2).SpeedSet=25;
Car(3).SpeedSet=25;
Car(4).SpeedSet=25;
Car(5).SpeedSet=26;



% =========================================
% MPC目标函数相关变量初始化
% =========================================

global ymin ymax;
ymin=-3; %边界下限约束
ymax=6;  %边界上限约束

global X_predict Y_predict PHI_predict v_predict WheelAng_predict APF_Value X_real Y_real v_real;


X_predict = zeros(Np,1);
Y_predict = zeros(Np,1);
PHI_predict = zeros(Np,1);
v_predict = zeros(Np,1);
WheelAng_predict = zeros(Np,1);

APF_Value=zeros(Np+1,1);

Q_f=60;     % APF factor
X_f=6;
Y_f=11;
R_f=5;      % speed factor
Pa_f=60;    % accelerator factor
Pw_f=60;   % wheelangacceleratoror factor

% =========================================
% 仿真参数记录
% =========================================

HostCar_Num=5;     %控制车辆数量
Cartemp=cell(N,1); %记录每个仿真步长中的各个车辆状态

%% 开始求解

for j=1:1:N
 tic   
     
    %计算第HostID辆车的控制输入量
    for i_Control=1:Num_Car
        if Car(i_Control).HostFlag==1
            %处理被控车辆
            [a_actual,deltaf_actual] = NewAPFMpc(Car,i_Control,Num_lane,lane,Np,Nc,Q_f,X_f,Y_f,R_f,Pa_f,Pw_f,Tp);
            CarNew(i_Control)=Car(i_Control);
            CarNew(i_Control) = NewCarSolve( Car(i_Control),a_actual,deltaf_actual,T);
            %保存被控车辆控制变量及参数
            CarNew(i_Control).a_real=a_actual;
            CarNew(i_Control).deltaf_real=deltaf_actual;
            CarNew(i_Control).v_real=CarNew(i_Control).State(4);
            CarNew(i_Control).wheelang_real=CarNew(i_Control).State(5);
        else
            %处理环境障碍车辆
            CarNew(i_Control)=Car(i_Control);
            CarNew(i_Control).State(1)=CarNew(i_Control).State(1)+T*CarNew(i_Control).State(4);
        end
    end
    
  
    Cartemp{j,1} = CarNew;
    Car=CarNew;

    fprintf('Update Process, j=%4.2f\n',j)
    for i_Control=1:Num_Car
        if Car(i_Control).HostFlag==1
            fprintf('Update Process, HostCar_Num=%4.2f\n',i_Control)
            fprintf('Update Process, HostCar_x=%4.2f\n',Car(i_Control).State(1))
            fprintf('Update Process, HostCar_y=%4.2f\n',Car(i_Control).State(2))
            fprintf('Update Process, HostCar_v=%4.2f\n',Car(i_Control).State(4))
            fprintf('Update Process, Max_APFValue=%6.2f\n',max(Car(i_Control).PredictState(:,6)))
        end
    end
   
toc

end

