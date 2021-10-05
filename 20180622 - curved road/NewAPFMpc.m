function [delta_v_actual,delta_f_actual] = NewAPFMpc(Car,HostID,Road,Np,Nc,Q_f,X_f,Y_f,R_f,Pa_f,Pw_f,intraDis,T)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    

    global  costvalue APF_Value X_error Y_error v_error;
    
    Nu=2;
    %获取当前时刻被控车辆
    [Carhost,Num_CoCar,CoCar,Num_obCar,obCar]=NewRelation(Car,HostID);
%     
%     %被控车辆策略检测
%     if Num_CoCar~=0
%         if Carhost.platoon == Carhost.ID
%             for St_i=1:1:Num_CoCar       % St_i short for Strategy i
%                 if ((CoCar(St_i).State(1) > Carhost.State(1) )&&( CoCar(St_i).State(1) - Carhost.State(1)<= 25 ) &&(abs(CoCar(St_i).State(2)-Carhost.State(2))<3))
%                     %找到前方潜在跟随车辆中横向距离最短的一辆
%                     Temp_yerror = abs(CoCar(St_i).State(2)-Carhost.State(2));   % 依次查询被控车辆前方协作车辆与
%                     if St_i ==1 || Min_yerror>= Temp_yerror
%                         Min_yerror=Temp_yerror;
%                         LeaderNum=St_i;
%                     end
%                     Carhost.platoon = CoCar(LeaderNum).ID;
%                 end
%             end
%             
%         end
%     end
    
    
    v_set=Carhost.SpeedSet;
    
    %权重矩阵设置
    
    Q_f=60;
     Q=zeros(Np+1,Np+1);
        for qi=1:1:(Np+1);
            for qj=1:1:(Np+1);
                if qi==qj
                    Q(qi,qj)=Q_f;
%                     Q_f=Q_f/1.05;
                    if (Carhost.PltnNum==1)
                        Q_f=Q_f/1.05;
                    else
                        Q_f=Q_f;  
                    end
                else
                    Q(qi,qj)=0; 
                end
            end
        end
    
    Q=Q_f*eye(Np+1,Np+1);
    Wx=X_f*eye(Np+1,Np+1);
    %     Wy=zeros(Np+1,Np+1);
%         for wy_i=1:1:(Np+1);
%             for wy_j=1:1:(Np+1);
%                 if wy_i==wy_j
%                     Wy(wy_i,wy_j)=Y_f;
%                     Y_f=Y_f/1.3;
%                 else
%                     Wy(wy_i,wy_j)=0; 
%                 end
%             end
%         end
    Wy=Y_f*eye(Np+1,Np+1);
%     Wy=zeros(Np+1,Np+1);
%         for wy_i=1:1:(Np+1);
%             for wy_j=1:1:(Np+1);
%                 if wy_i==wy_j
%                     Wy(wy_i,wy_j)=Y_f;
%                     Y_f=Y_f/1.3;
%                 else
%                     Wy(wy_i,wy_j)=0; 
%                 end
%             end
%         end
    R=R_f*eye(Np+1,Np+1);
    Pa=Pa_f*eye(Np,Np);
    Pw=Pw_f*eye(Np,Np);
    
    
    %不等式约束
    A_t=zeros(Nc,Nc);%见falcone论文 P181
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu));%对应于falcone论文约束处理的矩阵A,求克罗内克积
       
    Ut=kron(ones(Nc,1),[Carhost.State(4);Carhost.State(5)]); % 上一步实际输出控制量
    
    umin=[0.1*v_set;-0.43];%维数与控制变量的个数相同
    umax=[1.1*v_set;0.44];
%     delta_umin=[-1*T;-0.165*T];
    delta_umin=[-4.9*T;-0.165*T];
    delta_umax=[1*T;0.165*T];
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    A_cons_cell={A_I;-A_I};
    b_cons_cell={Umax-Ut;-Umin+Ut};
    A_cons=cell2mat(A_cons_cell);%（求解方程）状态量不等式约束增益矩阵，转换为绝对值的取值范围
    b_cons=cell2mat(b_cons_cell);%（求解方程）状态量不等式约束的取值
    
    % 状态量约束
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=delta_Umin;%（求解方程）状态量下界，包含控制时域内控制增量
    ub=delta_Umax;%（求解方程）状态量上界，包含控制时域内控制增量
    
    A=A_cons;    
    b=b_cons;
    Aeq=[];
    beq=[];

    options = optimset('Algorithm','active-set');
    X0=zeros(Nu*Nc,1);
    %% ==================================================
    % 生成障碍车辆在预测时域内的位置
    % ============================================================
    %生成无通信障碍车辆预测时域内位置    
%     for ob_i=1:1:Num_obCar    
%         obCar()
%         obCar(ob_i).PredictState(1,1:5)=obCar(ob_i).State;
%         for i=1:Np      
%             obCar(ob_i).PredictState(i+1,:)=obCar(ob_i).PredictState(i,:);
%             obCar(ob_i).PredictState(i+1,1)=obCar(ob_i).PredictState(i,1)+T*obCar(ob_i).PredictState(i,4);        
%         end    
%     end
    
    for ob_i=1:1:Num_obCar     
        obCar(ob_i).PredictRoadLocation(1,1:5) = obCar(ob_i).RoadLocation(1,1:5);
        obCar(ob_i).PredictState(1,1:5) = obCar(ob_i).State;
        obCar(ob_i).PredictRoadmilestore(1) = Roadloc(obCar(ob_i),Road );
        for i=1:Np-1
            obCar(ob_i).PredictState(i+1,1:5) = obCar(ob_i).PredictState(i,1:5);
            obCar(ob_i).PredictRoadLocation(i+1,:) = obCar(ob_i).PredictRoadLocation(i,1:5);
           
            s_temp = obCar(ob_i).PredictRoadLocation(i,3) + T * obCar(ob_i).PredictRoadLocation(i,4);
             
            if s_temp > Road(obCar(ob_i).PredictRoadLocation(i,1)).geometry.length
                obCar(ob_i).PredictRoadLocation(i+1,3) = s_temp - Road(obCar(ob_i).PredictRoadLocation(i,1)).geometry.length;
                obCar(ob_i).PredictRoadLocation(i+1,1) = obCar(ob_i).PredictRoadLocation(i,1)+1;                
            else
                obCar(ob_i).PredictRoadLocation(i+1,3) = s_temp;
            end
            CarState = CarLocation(obCar(ob_i).PredictRoadLocation(i+1,:),Road);    
            obCar(ob_i).PredictState(i+1,1:5)=[CarState.x CarState.y CarState.phi CarState.v CarState.wheelang];
            TempCar.RoadLocation = obCar(ob_i).PredictRoadLocation(i+1,1:5);
            obCar(ob_i).PredictRoadmilestore(i+1,:) = Roadloc(TempCar,Road );
        end          
    end 
    
    %获取可协作车辆预测位置
%     if Num_CoCar~=0
%         for co_i=1:1:Num_CoCar
%             CoCar(co_i).PredictState(1,1:5)=CoCar(co_i).State;
%             [row column]=size(CoCar(co_i).PredictState);
%             if row<Np+1 && row ~= 0
%                 for i=row+1:Np+1
%                     CoCar(co_i).PredictState(i,:)=CoCar(co_i).PredictState(i-1,:);
%                     CoCar(co_i).PredictState(i,1)=CoCar(co_i).PredictState(i-1,1)+T*CoCar(co_i).PredictState(i-1,4);   
%                 end
%             end            
%         end 
%     end
    if Num_CoCar~=0
        for co_i=1:1:Num_CoCar
%             CoCar(co_i).PredictState(1,1:5)=CoCar(co_i).State;
%             CoCar(co_i).PredictRoadLocation(1,1:5)=CoCar(co_i).RoadLocation;
%             CoCar(co_i).PredictRoadmilestore(1)=CoCar(co_i).PredictRoadmilestore;
%             [row column]=size(CoCar(co_i).PredictState);
%             if row<Np+1 && row ~= 0
%                 for i=row+1:Np+1
%                     CoCar(co_i).PredictState(i,:)=CoCar(co_i).PredictState(i-1,:);
%                     CoCar(co_i).PredictState(i,1)=CoCar(co_i).PredictState(i-1,1)+T*CoCar(co_i).PredictState(i-1,4);   
%                 end
%             end            
        end 
    end
    
    %打包环境事件 
    Num_obCarPredict=Num_CoCar+Num_obCar;
    obCarPredict=[CoCar,obCar];
    
    options=optimset('Algorithm','active-set');
    [Z,fval,exitflag]=fmincon(@(x)NewMY_costfunction(x,Carhost,Np,Nc,T,Num_obCarPredict,obCarPredict,Road,v_set,Q,Wx,Wy,R,Pa,Pw,intraDis),X0,A,b,Aeq,beq,lb,ub,[],options);%有约束求解，但速度慢
    
%     if ((max(APF_Value(1:10))>3.5)  && (Carhost.PltnNum==1))
%         delta_v_actual=delta_umin(1)/T;
%         delta_f_actual=-0.0001;
%     elseif ((max(APF_Value(11:end))>3.5)  && (Carhost.PltnNum==1))
%         delta_v_actual=delta_umin(1)/T;
%         delta_f_actual=Z(2)/T;
        
    if ((max(APF_Value)>6)  && (Carhost.PltnNum==1))
    
        delta_v_actual=delta_umin(1)/T;
        delta_f_actual=-0.0001;
        delta_f_actual=Z(2)/T;
        
    elseif ((max(APF_Value)>4)  && (Carhost.PltnNum==1))
        delta_v_actual=delta_umin(1)/(2*T);
        delta_f_actual=Z(2)/T;
    elseif ((max(APF_Value)>9)  && (Carhost.PltnNum~=1))
    
        delta_v_actual=delta_umin(1)/T;
        delta_f_actual=-0.0001;
        
    elseif ((max(APF_Value)>4.5)  && (Carhost.PltnNum~=1))
        delta_v_actual=delta_umin(1)/(2*T);
        delta_f_actual=Z(2)/T;
        
    else    
        delta_v_actual=Z(1)/T;
        delta_f_actual=Z(2)/T;                             % 求解得到的控制序列
    end
    
%     fprintf('Update Process, HostCar_Num=%4.2f\n',HostID)
% %     fprintf('Update Process, X_error=%6.2f\n',X_error'*Wx*X_error)
%     fprintf('Update Process, Y_error=%6.2f\n',Y_error'*Wy*Y_error)
% %     fprintf('Update Process, V_error=%6.2f\n',v_error'*R*v_error)
%     fprintf('Update Process, APFValue=%6.2f\n',APF_Value'*Q*APF_Value) 
    
          
    
end

