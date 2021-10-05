N=2500;

continue_j=j;

 Car=Cartemp{continue_j-1};
% Car(9).comEn=1;
% Car(1).PltnEn=0;
% Car(1).SetlaneEn=1;
% Car(1).Setlane=1.5;

% X_f=5;
% Y_f=20;
%   Car(3).PltnEn=0;
%  Car(4).PltnLength=1;
%  Car(6).State(1)= Car(6).State(1)-20;
%  Car(6).State(2)= 4.5;
% Car(4).platoon=4;
% Car(4).PltnNum=1;
% Car(5).platoon=4;
% Car(5).PltnNum=2;

% Car(1).SpeedSet=29;
%  Car(4).SpeedSet=28;
%  Car(5).SpeedSet=28;
% Car(7).HostFlag=0;
% Car(7).comEn=0;
% Car(7).SpeedSet=20;

for j=continue_j:1:N
  tic   
%    if j==100
%      Car(9).Setlane=1.5;
%      Car(9).SpeedSet=22;
%    end
     
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
            CarNew(i_Control).Roadmilestore = Roadloc(CarNew(i_Control),Road );
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
            fprintf('    phi=%4.2f',Car(i_Control).State(3)*180/pi)
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