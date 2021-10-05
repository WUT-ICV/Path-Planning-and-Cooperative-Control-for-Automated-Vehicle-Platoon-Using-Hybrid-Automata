input('Hey u idiot,r u sure u wannna Begin?');
clear all;
%% ==============================================
%   ��ʼ������������ϵͳ����������״̬�����������ϰ���Ϣ����
%  =====================================================
%ϵͳ������ʼ��
N=2000;   %ϵͳ���沽��
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

% Car(1).State=[48       -1.5   0   25     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
% Car(2).State=[100      4.5   0   20     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
% Car(3).State=[80      1.5   0   22     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
% Car(4).State=[100     -1.5   0   20     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�

Car(1).State=[80       4.5   0    22     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
Car(2).State=[80       1.5   0    20     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
Car(3).State=[80      -1.5   0    20     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
Car(4).State=[48       4.5   0    25     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�

[temp_m,Num_Car]=size(Car);
%��ʼ����������ΪĬ��ֵ
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

%���س�������
Car(4).HostFlag=1;

%��ͨ�ų�������

Car(4).comEn=1;
Car(4).comDelay=0.02;

%���Ӳ�������


%���س�����������
Car(4).SpeedSet=26;

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

Q_f=60;     % APF factor
X_f=6;
Y_f=11;
R_f=5;      % speed factor
Pa_f=60;    % accelerator factor
Pw_f=60;   % wheelangacceleratoror factor

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
            [a_actual,deltaf_actual] = NewAPFMpc(Car,i_Control,Num_lane,lane,Np,Nc,Q_f,X_f,Y_f,R_f,Pa_f,Pw_f,Tp);
            CarNew(i_Control)=Car(i_Control);
            CarNew(i_Control) = NewCarSolve( Car(i_Control),a_actual,deltaf_actual,T);
            %���汻�س������Ʊ���������
            CarNew(i_Control).a_real=a_actual;
            CarNew(i_Control).deltaf_real=deltaf_actual;
            CarNew(i_Control).v_real=CarNew(i_Control).State(4);
            CarNew(i_Control).wheelang_real=CarNew(i_Control).State(5);
        else
            %�������ϰ�����
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