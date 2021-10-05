

HostID=3;
%ColorSet='b--'
jjj=N-1;

plotrange=480;

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

PlotNum=10;

language=1;% 1 English//0 Chinese

fliterNum=20;
Car=Cartemp{1,1};
for i_Control=1:PlotNum
        if Car(i_Control).HostFlag==1
            HostNum=HostNum+1;
            for i=1:plotrange
                if i==651
                    i=651;
                end
                a_real(i,1)=0;
                if i<=fliterNum
                    a_real(i,1)=0;
                    for iii=1:fliterNum
                        a_real(i,1)=a_real(i,1)+Cartemp{iii}(i_Control).a_real;
                    end
                    a_real(i,1)=a_real(i,1)/fliterNum;
                else 
                    a_real(i,1)=0;
                    for iii=i-fliterNum+1:i
                        a_real(i,1)=a_real(i,1)+Cartemp{iii}(i_Control).a_real;
                    end
                    a_real(i,1)=a_real(i,1)/fliterNum;
                end
                   
%                 aa_real(i,1)=Cartemp{i}(i_Control).a_real;
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
            
            sss=0; %如果画图车辆不是从1车开始，则sss取前方车辆数，如sss=2 则是从3车开始画图
            if i_Control ==1+sss
                ColorSet=['r'];
                LineSet=['-.'];                
            elseif i_Control ==2+sss
                ColorSet=[0.8 0.5 0.3];
                LineSet=['-']; 
            elseif i_Control ==3+sss
                ColorSet=['g'];
                LineSet=['-.']; 
            elseif i_Control ==4+sss
                ColorSet=['b'];
                LineSet=['--']; 
            elseif i_Control ==5+sss
                ColorSet=['m'];
                LineSet=['-.']; 
            elseif i_Control ==6+sss
                ColorSet=['c'];
                LineSet=['-']; 
            elseif i_Control ==7+sss
                ColorSet=['y'];
                LineSet=['-.']; 
            elseif i_Control ==8+sss
                ColorSet=['k'];
                LineSet=['--']; 
            end
%             ColorSet=[rand(),rand(),rand()];
                      
            figure(3)
            plot([0,(plotrange-1)*T],[delta_umin(1)/Tp,delta_umin(1)/Tp],'Color',[0.6 0.6 0.6],'linestyle','--');
            hold on;
            plot([0,(plotrange-1)*T],[delta_umax(1)/Tp+0.05,delta_umax(1)/Tp+0.05],'Color',[0.6 0.6 0.6],'linestyle','--');
            hold on;
            Acc(HostNum)=plot(0:T:(plotrange-1)*T,a_real(1:1:plotrange,1),'Color',ColorSet,'linestyle',LineSet);            
            hold on;            
            axis([0 (plotrange-1)*T delta_umin(1)/Tp-(delta_umax(1)/Tp+0.05-delta_umin(1)/Tp)/2 delta_umax(1)/Tp+0.05+(delta_umax(1)/Tp+0.05-delta_umin(1)/Tp)/2]);
            
%             if language == 0
%                 title('加速度-时间曲线');
%             else
%                 title('Acceleration-time');
%             end
            
            if HostNum<=4      
                legend(Acc,'Car1','Car2','Car3', 'Car4'); 
            elseif i_Control==8+sss
                legend(Acc(1:4),'Car1','Car2','Car3', 'Car4','Orientation','horizontal');   
                legend BOXOFF;
%                 ahAcc=axes('position',get(gca,'position'),'visible','off');
%                 legend(ahAcc,Acc(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal'); 
                ahAcc=axes('position',[0.1300 0.0663 0.7750 0.8087],'visible','off');
                legend(ahAcc,Acc(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');
                set(gca,'Fontname','times new Roman');
                legend BOXOFF;
                ahAcc=axes('position',[0.1300 0.1163 0.7750 0.8087],'visible','off');
            end            
            if language == 0
                xlabel('时间 （s）');
                ylabel('加速度 （m/s^2）');
            else
                xlabel('time (s)');
                ylabel('acceleration (m/s^2)');
            end
            set(gca,'Fontname','Times New Roman');
            set(get(gca,'Title'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'XLabel'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'YLabel'),'FontSize',14);
%             grid on;
            hold on;
            
            figure(4)
            plot([0,(plotrange-1)*T],[(delta_umin(2)/pi*180)/Tp,(delta_umin(2)/pi*180)/Tp],'Color',[0.6 0.6 0.6],'linestyle','--');
            hold on;
            plot([0,(plotrange-1)*T],[(delta_umax(2)/pi*180)/Tp,(delta_umax(2)/pi*180)/Tp],'Color',[0.6 0.6 0.6],'linestyle','--');
            hold on;
            Del(HostNum)=plot(0:T:(plotrange-1)*T,deltaf_real(1:1:plotrange)/pi*180,'Color',ColorSet,'linestyle',LineSet);
            hold on;            
            axis([0 (plotrange-1)*T (delta_umin(2)/pi*180)/Tp-((delta_umax(2)/pi*180)/Tp-(delta_umin(2)/pi*180)/Tp)/2 (delta_umax(2)/pi*180)/Tp+((delta_umax(2)/pi*180)/Tp-(delta_umin(2)/pi*180)/Tp)/2]);
%             title('前轮偏转角速度\Delta\delta-时间曲线');
            if HostNum<=4      
                legend(Del,'Car1','Car2','Car3', 'Car4'); 
            elseif i_Control==8+sss
                legend(Del(1:4),'Car1','Car2','Car3', 'Car4','Orientation','horizontal');   
                legend BOXOFF;
%                 ahAcc=axes('position',get(gca,'position'),'visible','off');
%                 legend(ahAcc,Acc(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal'); 
                ahDel=axes('position',[0.1300 0.0663 0.7750 0.8087],'visible','off');
                legend(ahDel,Del(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman');
                legend BOXOFF;
                ahDel=axes('position',[0.1300 0.1163 0.7750 0.8087],'visible','off');
            end                    
             if language == 0
                xlabel('时间 （s）');
                ylabel('\Delta\delta (°/s)');
            else
                xlabel('time (s)');
                ylabel('\Delta\delta (°/s)');
            end
            set(gca,'Fontname','times new Roman');
            set(get(gca,'Title'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'XLabel'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'YLabel'),'FontSize',14);
            hold on;
            figure(5)
            Spd(HostNum)=plot(0:T:(plotrange-1)*T,v_real(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
            hold on;
            %plot([0,(plotrange-1)*T],[umin(1),umin(1)],'Color',[0.6 0.6 0.6],'linestyle','--');
            %hold on;
            %plot([0,(plotrange-1)*T],[umax(1),umax(1)],'Color',[0.6 0.6 0.6],'linestyle','--');
            %hold on;
            axis([0 (plotrange-1)*T 10 umax(1)+(umax(1)-umin(1))/2]); 
            axis([0 (plotrange-1)*T 20 35]);  %20170602
%             if language == 0
%                 title('速度-时间曲线');
%             else
%                 title('Speed-time');
%             end
            if HostNum<=4      
                legend(Spd,'Car1','Car2','Car3', 'Car4'); 
            elseif i_Control==8+sss
                legend(Spd(1:4),'Car1','Car2','Car3', 'Car4','Orientation','horizontal');   
                legend BOXOFF;
%                 ahAcc=axes('position',get(gca,'position'),'visible','off');
%                 legend(ahAcc,Acc(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman'); 
                ahSpd=axes('position',[0.1300 0.0663 0.7750 0.8087],'visible','off');
                legend(ahSpd,Spd(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman');
                legend BOXOFF;
                ahSpd=axes('position',[0.1300 0.1163 0.7750 0.8087],'visible','off');
            end 
            if language == 0
                xlabel('时间 （s）');
                ylabel('速度 （m/s）');
            else
                xlabel('time (s)');
                ylabel('speed (m/s)');
            end
            set(gca,'Fontname','times new Roman');
            set(get(gca,'Title'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'XLabel'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'YLabel'),'FontSize',14);
            hold on;
            figure(6)
            ang(HostNum)=plot(0:T:(plotrange-1)*T,wheelang_real(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
            hold on;
            plot([0,(plotrange-1)*T],[umin(2),umin(2)],'Color',[0.6 0.6 0.6],'linestyle','--');
            hold on;
            plot([0,(plotrange-1)*T],[umax(2),umax(2)],'Color',[0.6 0.6 0.6],'linestyle','--');
            hold on;
            axis([0 (plotrange-1)*T umin(2)-(umax(2)-umin(2))/2 umax(2)+(umax(2)-umin(2))/2]);
%             title('前轮转角-时间曲线');
            if HostNum<=4      
                legend(ang,'Car1','Car2','Car3', 'Car4'); 
            elseif i_Control==8+sss
                legend(ang(1:4),'Car1','Car2','Car3', 'Car4','Orientation','horizontal');   
                legend BOXOFF;
%                 ahAcc=axes('position',get(gca,'position'),'visible','off');
%                 legend(ahAcc,Acc(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman'); 
                ahang=axes('position',[0.1300 0.0663 0.7750 0.8087],'visible','off');
                legend(ahang,ang(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman');
                legend BOXOFF;
                ahang=axes('position',[0.1300 0.1163 0.7750 0.8087],'visible','off');
            end 
            if language == 0
                xlabel('时间 （s）');
                ylabel('前轮转角（°）');
            else
                xlabel('time (s)');
                ylabel('Front Wheel Steering Angle(°)');
            end
            
            set(gca,'Fontname','times new Roman');
            set(get(gca,'Title'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'XLabel'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'YLabel'),'FontSize',14);
            hold on;
            
            figure(7)
            Platoon(HostNum)=plot(0:T:(plotrange-1)*T,platoonID(1:1:plotrange,1),'Color',ColorSet,'linestyle',LineSet);            
            hold on;
            axis([0 (plotrange-1)*T 0 9]);
%            title('车辆所在车队ID-时间曲线');
            if HostNum<=4      
                legend(Platoon,'Car1','Car2','Car3', 'Car4'); 
            elseif i_Control==8+sss
                legend(Platoon(1:4),'Car1','Car2','Car3', 'Car4','Orientation','horizontal');   
                legend BOXOFF;
%                 ahAcc=axes('position',get(gca,'position'),'visible','off');
%                 legend(ahAcc,Acc(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman'); 
                ahPlatoon=axes('position',[0.1300 0.0663 0.7750 0.8087],'visible','off');
                legend(ahPlatoon,Platoon(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman');
                legend BOXOFF;
                ahPlatoon=axes('position',[0.1300 0.1163 0.7750 0.8087],'visible','off');
            end 
            if language == 0
                xlabel('时间 （s）');
                ylabel('车队ID');
            else
                xlabel('time (s)');
                ylabel('Platoon ID');
            end
            
            set(gca,'Fontname','times new Roman');
            set(get(gca,'Title'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'XLabel'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'YLabel'),'FontSize',14);
            hold on;
            figure(8)
            Leader(HostNum)=plot(0:T:(plotrange-1)*T,LeaderID(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
            hold on;
            axis([0 (plotrange-1)*T 0 9]);
%             title('车队中跟随车ID-时间曲线');
            if HostNum<=4      
                legend(Leader,'Car1','Car2','Car3', 'Car4'); 
            elseif i_Control==8+sss
                legend(Leader(1:4),'Car1','Car2','Car3', 'Car4','Orientation','horizontal');   
                legend BOXOFF;
%                 ahAcc=axes('position',get(gca,'position'),'visible','off');
%                 legend(ahAcc,Acc(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman'); 
                ahLeader=axes('position',[0.1300 0.0663 0.7750 0.8087],'visible','off');
                legend(ahLeader,Leader(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman');
                legend BOXOFF;
                ahLeader=axes('position',[0.1300 0.1163 0.7750 0.8087],'visible','off');
            end 
            
            if language == 0
                xlabel('时间 （s）');
                ylabel('车队中跟随车ID');
            else
                xlabel('time (s)');
                ylabel('Car ID');
            end
            set(gca,'Fontname','times new Roman');
            set(get(gca,'Title'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'XLabel'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'YLabel'),'FontSize',14);
            hold on;
            
            figure(9)
            Num(HostNum)=plot(0:T:(plotrange-1)*T,PltnNum(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
            hold on;
            axis([0 (plotrange-1)*T 0 9]);
%             title('被控车辆所处车队次序-时间曲线');
            if HostNum<=4      
                legend(Num,'Car1','Car2','Car3', 'Car4'); 
            elseif i_Control==8+sss
                legend(Num(1:4),'Car1','Car2','Car3', 'Car4','Orientation','horizontal');   
                legend BOXOFF;
%                 ahAcc=axes('position',get(gca,'position'),'visible','off');
%                 legend(ahAcc,Acc(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman'); 
                ahNum=axes('position',[0.1300 0.0663 0.7750 0.8087],'visible','off');
                legend(ahNum,Num(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman');
                legend BOXOFF;
                ahNum=axes('position',[0.1300 0.1163 0.7750 0.8087],'visible','off');
            end 
              
            if language == 0
                xlabel('时间 （s）');
                ylabel('次序');      
            else
                xlabel('time (s)');
                ylabel('The Order of Platoon');
            end
            set(gca,'Fontname','times new Roman');
            set(get(gca,'Title'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'XLabel'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'YLabel'),'FontSize',14);
            hold on;
            
            figure(10)
            Len(HostNum)=plot(0:T:(plotrange-1)*T,PltnLen(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
            hold on;
            axis([0 (plotrange-1)*T 0 9]);
%             title('所在车队长度-时间曲线');
            if HostNum<=4      
                legend(Len,'Car1','Car2','Car3', 'Car4'); 
            elseif i_Control==8+sss
                legend(Len(1:4),'Car1','Car2','Car3', 'Car4','Orientation','horizontal');   
                legend BOXOFF;
%                 ahAcc=axes('position',get(gca,'position'),'visible','off');
%                 legend(ahAcc,Acc(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman'); 
                ahLen=axes('position',[0.1300 0.0663 0.7750 0.8087],'visible','off');
                legend(ahLen,Len(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman');
                legend BOXOFF;
                ahLen=axes('position',[0.1300 0.1163 0.7750 0.8087],'visible','off');
            end 
            if language == 0
                xlabel('时间 （s）');
                ylabel('车队长度（辆）');   
            else
                xlabel('time (s)');
                ylabel('The Length of Platoon');
            end
            
            set(gca,'Fontname','times new Roman');
            set(get(gca,'Title'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'XLabel'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'YLabel'),'FontSize',14);
            hold on;
          
            
%             figure(11)
%             for lane_i=1:1:Num_lane
%                 if (lane_i==1) || (lane_i==Num_lane)
%                     plot([0,(plotrange-1)*T],[lane(lane_i),lane(lane_i)],'Color',[0.9 0.9 0.9],'linestyle','-','LineWidth',3);
%                     hold on;      
%                 else
%                     plot([0,(plotrange-1)*T],[lane(lane_i),lane(lane_i)],'Color',[0.9 0.9 0.9],'linestyle','--','LineWidth',3);
%                     hold on;
%                 end
%             end
%             pos(HostNum)=plot(0:T:(plotrange-1)*T, Y_pos(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
%             hold on;
%             axis([0 (plotrange-1)*T -4 7]);            
% %             if language == 0
% %                 title('Y坐标-时间轨迹');
% %             else
% %                 title('Y-coordinate - time');
% %             end
%             if HostNum<=4      
%                 legend(pos,'Car1','Car2','Car3', 'Car4'); 
%             elseif i_Control==8+sss
%                 legend(pos(1:4),'Car1','Car2','Car3', 'Car4','Orientation','horizontal');   
%                 legend BOXOFF;
% %                 ahAcc=axes('position',get(gca,'position'),'visible','off');
% %                 legend(ahAcc,Acc(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman'); 
%                 ahpos=axes('position',[0.1300 0.0663 0.7750 0.8087],'visible','off');
%                 legend(ahpos,pos(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman');
%                 legend BOXOFF;
%                 ahpos=axes('position',[0.1300 0.1163 0.7750 0.8087],'visible','off');
%             end 
% 
%             if language == 0
%                 xlabel('时间 （s）');
%                 ylabel('Y坐标 （m）');
%             else
%                 xlabel('time (s)');
%                 ylabel('Y-coordinate (m)');
%             end
%             set(gca,'Fontname','times new Roman');
%             set(get(gca,'Title'),'FontSize',14);%图上文字为8 point或小5号
%             set(get(gca,'XLabel'),'FontSize',14);%图上文字为8 point或小5号
%             set(get(gca,'YLabel'),'FontSize',14);
%             hold on;
            
            
            figure(12)
            headdis(HostNum)=plot(0:T:(plotrange-1)*T, headDis(1:1:plotrange),'Color',ColorSet,'linestyle',LineSet);
            hold on;
            axis([0 (plotrange-1)*T 0 20]);
            axis([0 (plotrange-1)*T 8 16]);%20170602
            
%             if language == 0
%                 title('与前车距离-时间曲线');
%             else
%                 title('Headway-time');
%             end
            if HostNum<=4      
                legend(headdis,'Car1','Car2','Car3', 'Car4'); 
            elseif i_Control==8+sss
                legend(headdis(1:4),'Car1','Car2','Car3', 'Car4','Orientation','horizontal');   
                legend BOXOFF;
%                 ahAcc=axes('position',get(gca,'position'),'visible','off');
%                 legend(ahAcc,Acc(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman'); 
                ahheaddis=axes('position',[0.1300 0.0663 0.7750 0.8087],'visible','off');
                legend(ahheaddis,headdis(5:8),'Car5','Car6','Car7', 'Car8','Orientation','horizontal');set(gca,'Fontname','times new Roman');
                legend BOXOFF;
                ahheaddis=axes('position',[0.1300 0.1163 0.7750 0.8087],'visible','off');
            end 

            if language == 0
                xlabel('时间 （s）');
                ylabel('与前车距离 （m）');
            else
                xlabel('time (s)');
                ylabel('Headway (m)');
            end
            set(gca,'Fontname','times new Roman');
            set(get(gca,'Title'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'XLabel'),'FontSize',14);%图上文字为8 point或小5号
            set(get(gca,'YLabel'),'FontSize',14);
            hold on;
 
            
        end
end






