input('Hey u idiot,r u sure u wannna Begin?');
clear all;
%% ==============================================
%   ��ʼ������������ϵͳ����������״̬�����������ϰ���Ϣ����
%  =====================================================
%ϵͳ������ʼ��
N=400;   %ϵͳ���沽��
T=0.02;   %ϵͳ��������
Tp=0.25;  %Ԥ�����Ԥ������
Np=25;
Nc=2;

%����������ʼ�����ã�������賵������ֱ�ߣ�ƽ����x�ᣩ
Num_lane=4;   %����������
lane=zeros(Num_lane,1);
lane(1)=-3;   %������������
lane(2)=0;    %������������
lane(3)=3;    %������������
lane(4)=6;    %������������

%% ==============================================
%   ����״̬��ʼ������
%  =====================================================


%����״̬����

Car(1).State=[42.55  1.5   0    23.27     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
Car(2).State=[31.91  1.5   0    23.27     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
Car(3).State=[21.28  1.5   0    23.27     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
Car(4).State=[10.64  1.5   0    23.27     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
Car(5).State=[0      1.5   0    23.27     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�

[temp_m,Num_Car]=size(Car);
%��ʼ����������ΪĬ��ֵ
for temp_i=1:Num_Car
    Car(temp_i).length=4.8;
    Car(temp_i).width=1.8;
    Car(temp_i).platoon=temp_i;
    Car(temp_i).PltnNum=1;
    Car(temp_i).ID=temp_i;
    Car(temp_i).PltnEn=1;  %ʹ���Ƿ���Լ�����������
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
end

%���س�������
Car(1).HostFlag=1;
Car(2).HostFlag=1;
Car(3).HostFlag=1;
Car(4).HostFlag=1;
Car(5).HostFlag=1;

%��ͨ�ų�������

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

%���Ӳ�������
% 
% Car(1).platoon=1;
% Car(1).PltnNum=1;
% Car(2).platoon=2;
% Car(2).LeaderID=2;
% Car(2).PltnNum=1;
% Car(2).PltnLength=2;
% Car(3).platoon=2;
% Car(3).LeaderID=2;
% Car(3).PltnNum=2;
% Car(3).PltnLength=2;

%���س�����������
% Car(1).SpeedSet=23;
% Car(2).SpeedSet=28;
% Car(3).SpeedSet=28;
%  Car(3).SpeedSet=28;
%  Car(4).SpeedSet=28;
%  Car(5).SpeedSet=28;

% =========================================
% MPCĿ�꺯����ر�����ʼ��
% =========================================

global ymin ymax;
ymin=-3; %�߽�����Լ��
ymax=6;  %�߽�����Լ��

global X_predict Y_predict PHI_predict v_predict WheelAng_predict APF_Value X_real Y_real v_real;


X_predict = zeros(Np,1);
Y_predict = zeros(Np,1);
PHI_predict = zeros(Np,1);
v_predict = zeros(Np,1);
WheelAng_predict = zeros(Np,1);

APF_Value=zeros(Np+1,1);

Q_f=60;       % APF factor
X_f=10;
Y_f=15;
R_f=5;       % speed factor
Pa_f=60;      % accelerator factor
Pw_f=60;      % wheelangacceleratoror factor

% =========================================
% ���������¼
% =========================================

Cartemp=cell(N,1); %��¼ÿ�����沽���еĸ�������״̬

%% ��ʼ���

for j=1:1:N
 tic   
 if j==201
     Car(1).SpeedSet=26;
 end

    %�����HostID�����Ŀ���������
     for i_Control=1:Num_Car
        if Car(i_Control).HostFlag==1
            %�������س���
            [a_actual,deltaf_actual] = NewAPFMpc(Car,i_Control,Num_lane,lane,Np,Nc,Q_f,X_f,Y_f,R_f,Pa_f,Pw_f,Car(i_Control).intraDis,Tp);          
            CarNew(i_Control)=Car(i_Control);
            CarNew(i_Control) = NewCarSolve( Car(i_Control),a_actual,deltaf_actual,T);
            %���汻�س������Ʊ���������
            CarNew(i_Control).a_real=a_actual;
            CarNew(i_Control).deltaf_real=deltaf_actual;
            CarNew(i_Control).v_real=CarNew(i_Control).State(4);
            CarNew(i_Control).wheelang_real=CarNew(i_Control).State(5);
        else
            %���������ϰ�����
            CarNew(i_Control)=Car(i_Control);
            CarNew(i_Control).State(1)=CarNew(i_Control).State(1)+T*CarNew(i_Control).State(4);
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
            fprintf('    x=%4.2f',Car(i_Control).State(1))
            fprintf('    y=%4.2f',Car(i_Control).State(2))
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