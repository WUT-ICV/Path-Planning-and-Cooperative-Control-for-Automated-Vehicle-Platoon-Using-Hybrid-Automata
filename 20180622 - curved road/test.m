%����������ʼ�����ã�������賵������ֱ�ߣ�ƽ����x�ᣩ
Num_lane=4;   %����������
lane=zeros(Num_lane,1);
lane(1)=-3;   %������������
lane(2)=0;    %������������
lane(3)=3;    %������������
lane(4)=6;    %������������

Car(1).State=[0      1.5   0    22     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�
Car(2).State=[40     1.5   0    10     0];     % x y phi v WheelAng��ǰ��ƫ�ǣ�

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

Car(1).HostFlag=1;

%�Ƴ���������
    HostID=1;
    %�����Ƴ�����
    resolution=0.1; 

    CarPlot=Car;
    [Carhost,Num_CoCar,CoCar,Num_obCar,obCar]=NewRelation(CarPlot,HostID);
    Num_obCartemp=Num_CoCar+Num_obCar;
    obCartemp=[CoCar,obCar];
    
%�����·����

    LaneStart=floor(Carhost.State(1)-10);
    LaneEnd=floor(Carhost.State(1)+60);
    [X,Y]=meshgrid(LaneStart:resolution:LaneEnd,lane(1)-5:resolution:lane(Num_lane)+5);
    [m n]=size(X);%�õ���ʾ����ĳߴ�
    APF_Value_show=zeros(m,n);
    for n_i=1:1:n
        for m_i=1:1:m  
            APF_Value_show(m_i,n_i)=NewAPFPoint_calc(X(m_i,n_i),Y(m_i,n_i),Carhost,Num_obCartemp,obCartemp,Num_lane,lane);
            if APF_Value_show(m_i,n_i)<0
                m_i=m_i;
            end
        end
    end
    APF_Value_show(find(APF_Value_show>15))=15;

% figure(3);
%     mesh(X,Y,APF_Value_show);    
%     hidden on
%     hold on
%     axis equal;
%     axis([LaneStart LaneEnd lane(1)-2 lane(Num_lane)+2 0 15] );
%     az = -60;
%     el = 25;
%     view(az,el);   
%     
figure(2)
    plot(Y(:,1),APF_Value_show(:,470));
    set(gca,'XDir','reverse')%��X����ת
%     axis equal;
    axis([-3.5 6.5 0 20] );
    hold on

%     figure(4)
%     plot([-10:0.1:60],APF_Value_show(96,:));
% 
% %     axis equal;
%     axis([0 60 0 20] );
%     hold on
    
%     for i_Control=1:Num_Car
%         if CarPlot(i_Control).HostFlag==1
%             %�����س���
%             [Carhost,Num_CoCar,CoCar,Num_obCar,obCar]=NewRelation(CarPlot,i_Control);
%             Num_obCartemp=Num_CoCar+Num_obCar;
%             obCartemp=[CoCar,obCar];
%             
%             APFPointValue=NewAPFPoint_calc(Carhost.State(1),Carhost.State(2),Carhost,Num_obCartemp,obCartemp,Num_lane,lane);
%             
%             plot3(Carhost.State(1),Carhost.State(2),APFPointValue+1.5,'r*');
%             
%             
%             axis equal;
%             hold on;
%             %axis([0 LaneLength lane(1)-5 lane(Num_lane)+5 ])    
%             if i_Control == HostID
%                 for p=1:1:Np
%                     PridictAPFtemp=Carhost.PredictState(p,6);
%                     if PridictAPFtemp>20
%                         PridictAPFtemp=20;
%                     end
%                     plot3(Carhost.PredictState(p,1),Carhost.PredictState(p,2),PridictAPFtemp+16,'g*');
%                 end
%             else
%                 for p=1:1:Np
%                     PridictAPFtemp=Carhost.PredictState(p,6);
%                     if PridictAPFtemp>20
%                         PridictAPFtemp=20;
%                     end
%                      plot3(Carhost.PredictState(p,1),Carhost.PredictState(p,2),Carhost.PredictState(p,6)+16,'b*');
%                 end
%             end
%             
%             hold on;
%             axis equal;
%             axis([LaneStart LaneEnd lane(1)-2 lane(Num_lane)+2 0 37] );
%             az = -90;
%             el = 60;
%             view(az,el);          
%         end
%     end
%     
%     pause(0.05);
%     hold off;  
