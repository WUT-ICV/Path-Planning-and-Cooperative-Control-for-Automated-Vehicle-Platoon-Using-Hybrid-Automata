function cost = NewMY_costfunction(x,Car,Np,Nc,T,Num_obCar,obCar,Road,Vref,Q,Wx,Wy,R,Pa,Pw,intraDis)
cost =0; % ��ʼ��Ŀ�꺯��ֵ
l=2.7;     % �������
% =================================================
% �����ʼ����������ǰ״̬��Ԥ��״̬��״̬ƫ��Ϳ�����
% =================================================
global costvalue X_predict Y_predict PHI_predict v_predict WheelAng_predict APF_Value X_real Y_real PHI_real v_real WheelAng_real X_error Y_error v_error Setlane_error;

X = Car.State(1);         % X����
Y = Car.State(2);         % Y����
PHI = Car.State(3);       % �����
V = Car.State(4);         % ��ʼ�ٶ�
WheelAng = Car.State(5);  % �����

X_predict = zeros(Np,1);
Y_predict = zeros(Np,1);
PHI_predict = zeros(Np,1);
v_predict = zeros(Np,1);
WheelAng_predict = zeros(Np,1);

X_error = zeros(Np+1,1);
Y_error = zeros(Np+1,1);
v_error = zeros(Np+1,1);

a=zeros(Np,1);  % ������ ���ٶ�
delta_f=zeros(Np,1); % ������ ���������

U1 = zeros(Np,1); % ��������delt_U
U2 = zeros(Np,1); % ��������delt_U

platoon_flag=0;


% ״̬����
% ==================================================
for i=1:1:Np
    if i==1
        a(i,1)=x(1);
        delta_f(i,1)=x(2); 
        X_predict(i,1) = X+T*(V+a(i,1))*cos(PHI);
        Y_predict(i,1) = Y+T*(V+a(i,1))*sin(PHI);
        PHI_predict(i,1) = PHI+T*(V+a(i,1))*tan(WheelAng+delta_f(i,1))/l;
        v_predict(i,1) = V+a(i,1);
        WheelAng_predict(i,1) = WheelAng+delta_f(i,1);
    elseif i~=1&&i<Nc
        a(i,1)=x(2*i-1);
        delta_f(i,1)=x(2*i);       
        X_predict(i,1) = X_predict(i-1)+T*(v_predict(i-1)+a(i,1))*cos(PHI_predict(i-1));
        Y_predict(i,1) = Y_predict(i-1)+T*(v_predict(i-1)+a(i,1))*sin(PHI_predict(i-1));
        PHI_predict(i,1) = PHI_predict(i-1)+T*(v_predict(i-1)+a(i,1))*tan(WheelAng_predict(i-1))/l;
        v_predict(i,1) = v_predict(i-1)+a(i,1);
        WheelAng_predict(i,1) = WheelAng_predict(i-1)+delta_f(i,1);
    else
        a(i,1)=x(2*Nc-1);
        delta_f(i,1)=x(2*Nc);       
        X_predict(i,1) = X_predict(i-1)+T*(v_predict(i-1)+a(i,1))*cos(PHI_predict(i-1));
        Y_predict(i,1) = Y_predict(i-1)+T*(v_predict(i-1)+a(i,1))*sin(PHI_predict(i-1));
        PHI_predict(i,1) = PHI_predict(i-1)+T*(v_predict(i-1)+a(i,1))*tan(WheelAng_predict(i-1))/l;
        v_predict(i,1) = v_predict(i-1)+a(i,1);
        WheelAng_predict(i,1) = WheelAng_predict(i-1)+delta_f(i,1);
    end
    
   %% ===================================================
    % ����Ԥ��ʱ�����Ƴ�ֵ
    % ============================================================
    
    X_real = zeros(Np+1,1);
    Y_real = zeros(Np+1,1);
    v_real = zeros(Np+1,1);
    X_real(1,1)=X;
    X_real(2:Np+1,1)=X_predict;
    Y_real(1,1)=Y;
    Y_real(2:Np+1,1)=Y_predict;
    PHI_real(1,1)=PHI;
    PHI_real(2:Np+1,1)=PHI_predict;
    v_real(1,1)=V;
    v_real(2:Np+1,1)=v_predict;
    WheelAng_real(1,1)=WheelAng;
    WheelAng_real(2:Np+1,1)=WheelAng_predict;
    
  
    %test
    Car_real=Car;
    Car_real.State=[X_real(i,1) Y_real(i,1) PHI_real(i,1) v_real(i,1) WheelAng_real(i,1)];   
    %test
    for counter_i=1:Num_obCar
        obCarPredict(counter_i)=obCar(counter_i);
        obCarPredict(counter_i).State=obCar(counter_i).PredictState(i,1:5);
        obCarnow(counter_i)=obCar(counter_i);
    end
   %  APF_Value(i)=NewAPFPoint_calc(X_real(i,1),Y_real(i,1),Car,Num_obCar,obCarPredict,Num_lane,lane);
      APF_Value(i)=NewCostFunAPFPoint_calc(X_real(i,1),Y_real(i,1),PHI_real(i,1),v_real(i,1),Car,Num_obCar,obCarPredict,obCarnow,Road);
    
    for counter_i=1:Num_obCar
        if (obCarPredict(counter_i).platoon == Car.platoon && obCarPredict(counter_i).PltnNum == Car.PltnNum-1)
%         if (obCarPredict(counter_i).platoon == Car.platoon )
            obx(i)=obCarPredict(counter_i).PredictState(i,1);                  %��c_i���ϰ�����x����
            oby(i)=obCarPredict(counter_i).PredictState(i,2);                  %��c_i���ϰ�����y����
            obtheta(i)=obCarPredict(counter_i).PredictState(i,3);              %��c_i���ϰ����������

            %�Ѽ����Ƴ������꣨x,y���任���ϰ���������ϵ�µ㣨xr(c_i),yr(c_i)��
            X_error(i) = (X_real(i,1)-obx(i))*cos(obtheta(i))+(Y_real(i,1)-oby(i))*sin(obtheta(i))+intraDis;
            Y_error(i) = (Y_real(i,1)-oby(i))*cos(obtheta(i))-(X_real(i,1)-obx(i))*sin(obtheta(i));
            
%             X_error(i)=X_real(i,1)-obCarPredict(counter_i).PredictState(i,1)+intraDis;
%             Y_error(i)=Y_real(i,1)-obCarPredict(counter_i).PredictState(i,2);
            v_error(i)=2*(v_real(i)-obCarPredict(counter_i).PredictState(i,4));
            R=20*eye(Np+1,Np+1);
            platoon_flag=1;
        end
    end
    U1(i,1) = a(i,1);
    U2(i,1) = delta_f(i,1);

end

if platoon_flag~=1
    v_error=v_real-Vref;
end
if Car.SetlaneEn==1
    Setlane_error=Y_real-Car.Setlane;
else
    Setlane_error=Y_real-Y_real;
end


% ����Ŀ�꺯��ֵ
% =================================================
%cost = cost+Y_error'*R*Y_error+X_error'*Q*X_error; % ����Ԥ��ʱ����ÿһ��Ԥ��ƫ��֮��
cost = cost+APF_Value'*Q*APF_Value+Y_error'*Wy*Y_error+Setlane_error'*(2*Wy)*Setlane_error+X_error'*Wx*X_error+v_error'*R*v_error+U1'*Pa*U1+U2'*Pw*U2;
if costvalue> cost
    costvalue = cost;
end
%fprintf('Update Process, cost=%6.2f\n',cost)
%������� 
  
  
  

    
    