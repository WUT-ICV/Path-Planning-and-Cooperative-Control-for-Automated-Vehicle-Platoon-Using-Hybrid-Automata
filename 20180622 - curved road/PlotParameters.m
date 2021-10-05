 

HostID=1;
%ColorSet='b--'
jjj=N-1;

plotrange=3500;
plotrange=300;

HostNum=0;
Acc=[];
Spd=[];
Del=[];
ang=[];
pos=[];
headdis=[];

% platoon
Platoon=[];
Leader=[];
Num=[];
Len=[];

fliterNum=20;
Car=Cartemp{1,1};
for i_Control=1:Num_Car
        if Car(i_Control).HostFlag==1
            HostNum=HostNum+1;
            for i=1:plotrange
                a_real(i,1)=0;
                if i<=fliterNum
                    a_real(i,1)=0;
                    for iii=1:fliterNum
                        a_real(i,1)=a_real(i,1)+Cartemp{iii}(i_Control).a_real;
                    end
                    a_real(i,1)=a_real(i,1)/fliterNum;
                else 
                    for iii=i-fliterNum:i
                        a_real(i,1)=a_real(i,1)+Cartemp{iii}(i_Control).a_real;
                    end
                    a_real(i,1)=a_real(i,1)/fliterNum;
                end
                   
%               a_real(i,1)=Cartemp{i}(i_Control).a_real;
                deltaf_real(i,1)=Cartemp{i}(i_Control).deltaf_real;
                v_real(i,1)=Cartemp{i}(i_Control).v_real;
                wheelang_real(i,1)=Cartemp{i}(i_Control).wheelang_real;
                Y_pos(i,1)=Cartemp{i}(i_Control).State(2);
                %platoon parameters
                platoonID(i,1)=Cartemp{i}(i_Control).platoon;
                LeaderID(i,1)=Cartemp{i}(i_Control).LeaderID;
                PltnNum(i,1)=Cartemp{i}(i_Control).PltnNum;
                PltnLen(i,1)=Cartemp{i}(i_Control).PltnLength;
                if i_Control~=1
                    headDis(i,1)=Cartemp{i}(i_Control-1).State(1)-Cartemp{i}(i_Control).State(1);
                else
                    headDis(i,1)=100;
                end
                         
                
                
            end
            [Carhost,Num_CoCar,CoCar,Num_obCar,obCar]=NewRelation(Car,i_Control);
            v_set=Carhost.SpeedSet;
%             umin=[0.1*v_set;-0.43];%维数与控制变量的个数相同
%             umax=[1.1*v_set;0.44];
%             delta_umin=[-1*Tp;-0.165*Tp];
%             delta_umax=[1*Tp;0.165*Tp];
            
            umin=[0.1*v_set;-0.43];%维数与控制变量的个数相同
            umax=[1.1*v_set;0.44];

            delta_umin=[-4.9*Tp;-0.165*Tp];
            delta_umax=[1*Tp;0.165*Tp];
            
            sss=0;
            if i_Control ==1+sss
                ColorSet=['r'];
                LineSet=['-'];                
            elseif i_Control ==2+sss
                ColorSet=[1 0.5 0.1];
                LineSet=['-']; 
            elseif i_Control ==3+sss
                ColorSet=['g'];
                LineSet=['-.']; 
            elseif i_Control ==4+sss
                ColorSet=['b'];
                LineSet=['--']; 
            elseif i_Control ==5+sss
                ColorSet=['m'];
                LineSet=['-']; 
            elseif i_Control ==6+sss
                ColorSet=['c'];
                LineSet=[':']; 
            elseif i_Control ==7+sss
                ColorSet=['y'];
                LineSet=['-.']; 
            elseif i_Control ==8+sss
                ColorSet=['k'];
                LineSet=['--']; 
            end
%             ColorSet=[rand(),rand(),rand()];
                      
            figure(3)
            plot([0,N*T],[delta_umin(1)/Tp,delta_umin(1)/Tp],'Color',[0.6 0.6 0.6],'linestyle','--');
            hold on;
            plot([0,N*T],[delta_umax(1)/Tp+0.05,delta_umax(1)/Tp+0.05],'Color',[0.6 0.6 0.6],'linestyle','--');
            hold on;
            Acc(HostNum)=plot(0:T:(plotrange-1)*T,a_real(1:1:plotrange,1),'Color',ColorSet,'linestyle',LineSet);            
            hold on;            
            axis([0 N*T delta_umin(1)/Tp-(delta_umax(1)/Tp+0.05-delta_umin(1)/Tp)/2 delta_umax(1)/Tp+0.05+(delta_umax(1)/Tp+0.05-delta_umin(1)/Tp)/2]);
            title('加速度-时间曲线');
            legend(Acc,'Car1','Car2','Car3', 'Car4','Car5','Car6','Car7','Car8');
            xlabel('时间 （s）');
            ylabel('加速度 （m/s^2）');
            set(get(gca,'Title'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'XLabel'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'YLabel'),'FontSize',14);
            grid on;
            hold on;
            figure(4)
            Del(HostNum)=plot(0:T:(plotrange-1)*T,deltaf_real(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
            hold on;
            plot([0,N*T],[delta_umin(2)/Tp,delta_umin(2)/Tp],'Color',[0.6 0.6 0.6],'linestyle','--');
            hold on;
            plot([0,N*T],[delta_umax(2)/Tp,delta_umax(2)/Tp],'Color',[0.6 0.6 0.6],'linestyle','--');
            hold on;
            axis([0 N*T delta_umin(2)/Tp-(delta_umax(2)/Tp-delta_umin(2)/Tp)/2 delta_umax(2)/Tp+(delta_umax(2)/Tp-delta_umin(2)/Tp)/2]);
            title('\Delta\delta');
            legend(Del,'Car1','Car2','Car3', 'Car4','Car5','Car6','Car7','Car8');
            xlabel('Time /s');
            ylabel('\Delta\delta rad/s^2');
            hold on;
            figure(5)
            Spd(HostNum)=plot(0:T:(plotrange-1)*T,v_real(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
            hold on;
            %plot([0,N*T],[umin(1),umin(1)],'Color',[0.6 0.6 0.6],'linestyle','--');
            %hold on;
            %plot([0,N*T],[umax(1),umax(1)],'Color',[0.6 0.6 0.6],'linestyle','--');
            %hold on;
            axis([0 N*T 10 umax(1)+(umax(1)-umin(1))/2]);
            title('Speed');
            legend(Spd,'Car1','Car2','Car3', 'Car4','Car5','Car6','Car7','Car8');
            xlabel('Time /s');
            ylabel('Speed m/s');
            hold on;
            figure(6)
            ang(HostNum)=plot(0:T:(plotrange-1)*T,wheelang_real(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
            hold on;
            plot([0,N*T],[umin(2),umin(2)],'Color',[0.6 0.6 0.6],'linestyle','--');
            hold on;
            plot([0,N*T],[umax(2),umax(2)],'Color',[0.6 0.6 0.6],'linestyle','--');
            hold on;
            axis([0 N*T umin(2)-(umax(2)-umin(2))/2 umax(2)+(umax(2)-umin(2))/2]);
            title('Front Wheel angle');
            legend(ang,'Car1','Car2','Car3', 'Car4','Car5','Car6','Car7','Car8');
            xlabel('Time /s');
            ylabel('angle rad');
            hold on;
            
            figure(7)
            Platoon(HostNum)=plot(0:T:(plotrange-1)*T,platoonID(1:1:plotrange,1),'Color',ColorSet,'linestyle',LineSet);            
            hold on;
            axis([0 (plotrange-1)*T 0 9]);
            title('Platoon');
            legend(Platoon,'Car1','Car2','Car3', 'Car4','Car5','Car6','Car7','Car8');
            xlabel('Time /s');
            ylabel('PlatoonID');
            hold on;
            figure(8)
            Leader(HostNum)=plot(0:T:(plotrange-1)*T,LeaderID(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
            hold on;
            axis([0 (plotrange-1)*T 0 9]);
            title('LeaderID');
            legend(Leader,'Car1','Car2','Car3', 'Car4','Car5','Car6','Car7','Car8');
            xlabel('Time /s');
            ylabel('LeaderID');
            hold on;
            figure(9)
            Num(HostNum)=plot(0:T:(plotrange-1)*T,PltnNum(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
            hold on;
            axis([0 (plotrange-1)*T 0 9]);
            title('PltnNum');
            legend(Num,'Car1','Car2','Car3', 'Car4','Car5','Car6','Car7','Car8');
            xlabel('Time /s');
            ylabel('PltnNum');
            hold on;
            figure(10)
            Len(HostNum)=plot(0:T:(plotrange-1)*T,PltnLen(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
            hold on;
            axis([0 (plotrange-1)*T 0 9]);
            title('PltnLen');
            legend(Len,'Car1','Car2','Car3', 'Car4','Car5','Car6','Car7','Car8');
            xlabel('Time /s');
            ylabel('PltnLen');
            hold on;
          
            
            figure(11)
            pos(HostNum)=plot(0:T:(plotrange-1)*T, Y_pos(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
            hold on;
            axis([0 (plotrange-1)*T -3.5 6.5]);
            title('Y position');
            legend(pos,'Car1','Car2','Car3', 'Car4','Car5','Car6','Car7','Car8');
            xlabel('Time /s');
            ylabel('Y-Coodinate');
            hold on;
            
            
            figure(12)
            headdis(HostNum)=plot(0:T:(plotrange-1)*T, headDis(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
            hold on;
            axis([0 (plotrange-1)*T 0 50]);
            title('HeadDis');
            legend(headdis,'Car1','Car2','Car3', 'Car4','Car5','Car6','Car7','Car8');
            xlabel('Time /s');
            ylabel('HeadDis');
            hold on;
 
            
        end
end






