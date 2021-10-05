NN=259;
% figure_num=5;
figure_i=1  ;
tt=10;

if N==j
    NN=N;
else
    NN=j;
end
    
% NN=600;

LaneLength_plot_local=1800;

%Figure_frame=[1  500 800  1600  2400  3000];
Figure_frame=[1  50 100  150  200  250];
[temp,figure_num]=size(Figure_frame);



for jj=1:10:NN
    Car=Cartemp{jj,1};
    HostID=7;
    [Carhost,Num_CoCar,CoCar,Num_obCar,obCar]=NewRelation(Car,HostID);
    laneStart_plot=-100+Carhost.State(1);
    LaneLength_plot= Carhost.State(1)+100;
    
%     laneStart_plot=-100+Carhost.State(1);
%     LaneLength_plot= Carhost.State(1)+100;
    
    %分出被控车辆状态 
    
figure(4)
set(gcf, 'position', [0 0 750 300]);
[temp_m,Num_Section] = size(Road);
    for i= 1 : Num_Section
        for lane_i = 1:length(Road(i).lane)
            if Road(i).lane(lane_i).type == 0
                plot(Road(i).lane(lane_i).sample_x,Road(i).lane(lane_i).sample_y,'r-' );  
                hold on
%                 plot(Road(i).samplepoint.x,Road(i).samplepoint.y,'b-' );      
%                 hold on
            else
                plot(Road(i).lane(lane_i).sample_x,Road(i).lane(lane_i).sample_y,'r--' );
                hold on
            end      

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
%    axis([laneStart_plot LaneLength_plot lane(1)-2 lane(Num_lane)+2]);
    axis([laneStart_plot LaneLength_plot -40+Carhost.State(2) 40+Carhost.State(2)]);
    title(['T=',num2str(jj*T),' s']);
    xlabel('x /m');
    ylabel('y /m');
    pause(0.001);
    set(gcf,'color',[1 1 1]) %设置背景色为白色
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

    