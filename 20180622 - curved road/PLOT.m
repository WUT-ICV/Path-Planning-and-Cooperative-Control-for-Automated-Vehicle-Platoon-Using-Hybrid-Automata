NN=259;
% figure_num=5;
figure_i=1  ;
tt=10;

if N==j
    NN=N;
else
    NN=j;
end
    


LaneLength_plot_local=1800;

%Figure_frame=[1  500 800  1600  2400  3000];
Figure_frame=[1  50 100  150  200  250];
[temp,figure_num]=size(Figure_frame);



for jj=1:10:NN
    Car=Cartemp{jj,1};
    HostID=1;
    [Carhost,Num_CoCar,CoCar,Num_obCar,obCar]=NewRelation(Car,HostID);
    laneStart_plot=-100+Carhost.State(1);
    LaneLength_plot= Carhost.State(1)+100;
    
%     laneStart_plot=-100+Carhost.State(1);
%     LaneLength_plot= Carhost.State(1)+100;
    
    %分出被控车辆状态 

    if (jj==Figure_frame(1,figure_i))
        figure(2);
        subplot(figure_num,1,figure_i);
%             for lane_i=1:1:Num_lane
%                 if (lane_i==1) || (lane_i==Num_lane)
%                     plot([laneStart_plot,LaneLength_plot],[lane(lane_i),lane(lane_i)],'r-');
%                     hold on;      
%                 else
%                     plot([laneStart_plot,LaneLength_plot],[lane(lane_i),lane(lane_i)],'r--');
%                     hold on;
%                 end
%             end
            %画出被控车辆      
            [plotxx,plotyy]=plotCar(Carhost);
            fill(plotxx,plotyy,'r','edgealpha',0);
            hold on
            %画出协作车辆
            for ii=1:Num_CoCar
                [plotxx,plotyy]=plotCar(CoCar(ii));
                fill(plotxx,plotyy,'g','edgealpha',0);
                hold on                
            end 
            %画出障碍车辆
            for ii=1:Num_obCar
                [plotxx,plotyy]=plotCar(obCar(ii));
                fill(plotxx,plotyy,'b','edgealpha',0);
                hold on                
            end 
            axis equal;
            hold on
%             axis([laneStart_plot LaneLength_plot lane(1)-2 lane(Num_lane)+2]);
            axis([laneStart_plot LaneLength_plot -10 40]);
            title(['T=',num2str(Figure_frame(1,figure_i)*T),' s']);
            xlabel('x /m');
            ylabel('y /m');
            
            figure_i=figure_i+1;
            
        
    end
    
                                    figure(4)
                                    set(gcf, 'position', [0 0 1000 300]);
                                             for lane_i=1:1:Num_lane
                                                if lane(lane_i).type == 1
                                                    t_circ = - 2*pi/3 : pi/500 : - pi/4;
                                                    x = lane(lane_i).curv_r * cos(t_circ);
                                                    y = lane(lane_i).curv_r * sin(t_circ)+(lane(lane_i).curv_r+lane(lane_i).start);
                                                    plot(x,y,'r--');
                                                    hold on;      
                                                else
                                                    t_circ = - 2*pi/3 : pi/500 : - pi/4;
                                                    x = lane(lane_i).curv_r * cos(t_circ);
                                                    y = lane(lane_i).curv_r * sin(t_circ)+(lane(lane_i).curv_r+lane(lane_i).start);
                                                    plot(x,y,'r-');
                                                    hold on;   
                                                end
                                             end

                                             %画出层级被控车辆
                                            for ii=1:Num_Car
                                                
                                                if Car(ii).PltnNum==1 && Car(ii).HostFlag==1
                                                    [plotxx,plotyy]=plotCar(Car(ii));
                                                    fill(plotxx,plotyy,'r','edgealpha',0);                                                    
                                                    if (Car(ii).State(1)>laneStart_plot+5) && (Car(ii).State(1)<LaneLength_plot-5)
                                                        text(Car(ii).State(1)-1,Car(ii).State(2)+2.5,num2str(Car(ii).platoon));
                                                    end
                                                    hold on  
                                                elseif Car(ii).PltnNum~=1
                                                    [plotxx,plotyy]=plotCar(Car(ii));
                                                    fill(plotxx,plotyy,'g','edgealpha',0);
                                                    if Car(ii).State(1)>laneStart_plot+5 && Car(ii).State(1)<LaneLength_plot-5
                                                        text(Car(ii).State(1)-1,Car(ii).State(2)+2.5,num2str(Car(ii).platoon));
                                                    end
                                                    hold on 
                                                else
                                                    [plotxx,plotyy]=plotCar(Car(ii));
                                                    fill(plotxx,plotyy,'b','edgealpha',0);
                                                    if Car(ii).State(1)>laneStart_plot+5 && Car(ii).State(1)<LaneLength_plot-5
                                                        text(Car(ii).State(1)-1,Car(ii).State(2)+2.5,num2str(Car(ii).platoon));
                                                    end
                                                    hold on    
                                                end
                                                
                                            end
                                         
                                            axis equal;
                                            hold on
%                                             axis([laneStart_plot LaneLength_plot lane(1)-2 lane(Num_lane)+2]);
                                            axis([laneStart_plot LaneLength_plot -20+Carhost.State(2) 20+Carhost.State(2)]);
                                            title(['T=',num2str(jj*T),' s']);
                                            xlabel('x /m');
                                            ylabel('y /m');
                                            pause(0.001);
                                            frame=getframe(gcf);
                                            im=frame2im(frame);
                                            [I,map]=rgb2ind(im,256);
                                            
                                            
                                            hold off
                                            if jj==1
                                              imwrite(I,map,'ex3.gif','gif','Loopcount',1,'DelayTime',0.001);
                                            else
                                              imwrite(I,map,'ex3.gif','gif','WriteMode','append','DelayTime',0.001);
                                            end
            
   

    
end

    