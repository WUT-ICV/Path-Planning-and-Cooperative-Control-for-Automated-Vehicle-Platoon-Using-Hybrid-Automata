function Carhost = NewCarSolve(Carhost,delta_v,delta_f,T)
%UNTITLED Summary of this function goes here
global X_predict Y_predict PHI_predict v_predict WheelAng_predict APF_Value;
%   Detailed explanation goes here
    l=2.7;
    
    Noise_factor=0.1;
    delta_v=delta_v+(Noise_factor*delta_v*2*(rand()-0.5)); 
    delta_f=delta_f+(Noise_factor*delta_f*2*(rand()-0.5));  
 
    v_actual=Carhost.State(4)+delta_v*T;               % 求解得到的控制序列
    Wheelang_actual=Carhost.State(5)+delta_f*T;

    % 将求解得到的结果作用于系统
    X00(1)=Carhost.State(1);
    X00(2)=Carhost.State(2);
    X00(3)=Carhost.State(3);
    XOUT=dsolve('Dx-v_actual*cos(z)=0','Dy-v_actual*sin(z)=0','Dz-v_actual/l*tan(Wheelang_actual)=0','x(0)=X00(1)','y(0)=X00(2)','z(0)=X00(3)');
    t=T;
    Carhost.State(1)=eval(XOUT.x);
    Carhost.State(2)=eval(XOUT.y);
    Carhost.State(3)=eval(XOUT.z);
    Carhost.State(4)=v_actual;
    Carhost.State(5)=Wheelang_actual;   
    
    Carhost.PredictState=[               Carhost.State                                   APF_Value(1);
                              X_predict Y_predict PHI_predict v_predict WheelAng_predict APF_Value(2:end)];

end

