function Road = SetRoad(StartPoint,Sectionlength,Sectiontype,SectionCurve,lane,sample_factor)
%StarPoint = [x y hdg]
%Sectionlength = [length1 length2 length3 ...]
%Sectiontype = [type1 type2 type3 ...]
%SectionCurve = [CurveStart1 CurveEnd1;
%                 CurveStart2 CurveEnd2;
%                 ......]
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
Road(1).geometry.start = 0 ; % start length
Road(1).geometry.length = Sectionlength(1); %length
Road(1).geometry.SPoint = [StartPoint(1) StartPoint(2)]; % x y
Road(1).geometry.hdg = StartPoint(3); % hdg
Road(1).geometry.type = Sectiontype(1); %1 -> Line; 2->Spiral; 3->Curvature

[temp_m,Num_Section] = size(Sectionlength);
if Num_Section > 1
    for i=2:1:Num_Section
        Road(i).geometry.start = Road(i-1).geometry.start + Road(i-1).geometry.length;
        Road(i).geometry.length = Sectionlength(i);
        Road(i).geometry.type = Sectiontype(i);       % 1 -> Line; 2->Spiral; 3->Curvature
        Road(i).geometry.Curve = SectionCurve(i,:);     % CurvStart CurvEnd
        if Road(i-1).geometry.type == 1
            Road(i).geometry.SPoint = [Road(i-1).geometry.SPoint(1)+ Road(i-1).geometry.length * cos(Road(i-1).geometry.hdg) 
                                       Road(i-1).geometry.SPoint(2)+ Road(i-1).geometry.length * sin(Road(i-1).geometry.hdg)];            
            Road(i).geometry.hdg = Road(i-1).geometry.hdg; % hdg   
            Road(i).geometry.EPoint = [Road(i).geometry.SPoint(1)+ Road(i).geometry.length * cos(Road(i).geometry.hdg) 
                                       Road(i).geometry.SPoint(2)+ Road(i).geometry.length * sin(Road(i).geometry.hdg)];
            count_i=0;                                 
            for sample_length= 0:sample_factor:Road(i-1).geometry.length
                count_i=count_i+1;
                Road(i-1).samplepoint.x(count_i) = Road(i-1).geometry.SPoint(1)+ sample_length * cos(Road(i-1).geometry.hdg);                                       
                Road(i-1).samplepoint.y(count_i) = Road(i-1).geometry.SPoint(2)+ sample_length * sin(Road(i-1).geometry.hdg);
                Road(i-1).samplepoint.hdg(count_i) = Road(i-1).geometry.hdg;
            end
        elseif Road(i-1).geometry.type == 3
            a = 2 * Road(i-1).geometry.Curve(1) * sin (Road(i-1).geometry.length  / ( Road(i-1).geometry.Curve(1) * 2));
            alpha = (Road(i-1).geometry.length  / ( Road(i-1).geometry.Curve(1) * 2)) +  Road(i-1).geometry.hdg;
            Road(i).geometry.SPoint = [Road(i-1).geometry.SPoint(1) + a * cos(alpha) Road(i-1).geometry.SPoint(2) + a * sin(alpha)];
            Road(i).geometry.hdg = (Road(i-1).geometry.length  / ( Road(i-1).geometry.Curve(1) * 2))*2+Road(i-1).geometry.hdg; % hdg 
            count_i=0;
            for sample_length= 0:sample_factor:Road(i-1).geometry.length
                count_i=count_i+1;
                Sample_a = 2 * Road(i-1).geometry.Curve(1) * sin (sample_length / ( Road(i-1).geometry.Curve(1) * 2));
                SampleAlpha = sample_length  / ( Road(i-1).geometry.Curve(1) * 2) +  Road(i-1).geometry.hdg;
                Road(i-1).samplepoint.x(count_i) = Road(i-1).geometry.SPoint(1) + Sample_a * cos(SampleAlpha);
                Road(i-1).samplepoint.y(count_i) = Road(i-1).geometry.SPoint(2) + Sample_a * sin(SampleAlpha);
                Road(i-1).samplepoint.hdg(count_i) = (sample_length  / ( Road(i-1).geometry.Curve(1) * 2))*2+Road(i-1).geometry.hdg;
            end            
        end        
    end
end 
if Road(Num_Section).geometry.type == 1
    Road(Num_Section).geometry.EPoint = [Road(Num_Section).geometry.SPoint(1)+ Road(Num_Section).geometry.length * cos(Road(Num_Section).geometry.hdg) 
                                         Road(Num_Section).geometry.SPoint(2)+ Road(Num_Section).geometry.length * sin(Road(Num_Section).geometry.hdg)];
    count_i=0;                                 
    for sample_length= 0:sample_factor:Road(Num_Section).geometry.length
        count_i=count_i+1;
        Road(Num_Section).samplepoint.x(count_i) = Road(Num_Section).geometry.SPoint(1)+ sample_length * cos(Road(Num_Section).geometry.hdg);                                       
        Road(Num_Section).samplepoint.y(count_i) = Road(Num_Section).geometry.SPoint(2)+ sample_length * sin(Road(Num_Section).geometry.hdg);
        Road(Num_Section).samplepoint.hdg(count_i) = Road(Num_Section).geometry.hdg;
    end
    
elseif Road(Num_Section).geometry.type == 3

    count_i=0;
    for sample_length= 0:sample_factor:Road(Num_Section).geometry.length
        count_i=count_i+1;
        Sample_a = 2 * Road(Num_Section).geometry.Curve(1) * sin (sample_length / ( Road(Num_Section).geometry.Curve(1) * 2));
        SampleAlpha = sample_length  / ( Road(Num_Section).geometry.Curve(1) * 2) +  Road(Num_Section).geometry.hdg;
        Road(Num_Section).samplepoint.x(count_i) = Road(Num_Section).geometry.SPoint(1) + Sample_a * cos(SampleAlpha);
        Road(Num_Section).samplepoint.y(count_i) = Road(Num_Section).geometry.SPoint(2) + Sample_a * sin(SampleAlpha);
        Road(Num_Section).samplepoint.hdg(count_i) = (sample_length  / ( Road(Num_Section).geometry.Curve(1) * 2))*2+Road(Num_Section).geometry.hdg;
    end            
end
% 计算每条lane的采样点
for i=1:Num_Section
    Road(i).SampleFactor = sample_factor;
    Road(i).lane=lane;   
    Road_width = 0;
    for lane_i=1:1:length(Road(i).lane)
        Road_width = Road_width + Road(i).lane(lane_i).width;
    end
    Road(i).width = Road_width;
    
    if Road(i).geometry.type == 1 
        StartPoint = [Road(i).geometry.SPoint(1) Road(i).geometry.SPoint(2)];
        EndPoint = [Road(i).geometry.SPoint(1)+ Road(i).geometry.length * cos(Road(i).geometry.hdg) 
                     Road(i).geometry.SPoint(2)+ Road(i).geometry.length * sin(Road(i).geometry.hdg)];   
        lane(1).StartPoint = StartPoint;
        lane(1).EndPoint = EndPoint;
        for lane_i=2:1:length(Road(i).lane)
            width=Road(i).lane(lane_i).width;
            lane(lane_i).StartPoint = [lane(lane_i-1).StartPoint(1)+ width*cos(Road(i).geometry.hdg-pi/2) lane(lane_i-1).StartPoint(2)+ width*sin(Road(i).geometry.hdg-pi/2)];
            lane(lane_i).EndPoint = [lane(lane_i-1).EndPoint(1)+ width*cos(Road(i).geometry.hdg-pi/2) lane(lane_i-1).EndPoint(2)+ width*sin(Road(i).geometry.hdg-pi/2)];            
        end
        
        for lane_i=1:length(Road(i).lane)
            count_i=0;
            for sample_length= 0:sample_factor:Road(i).geometry.length
                count_i=count_i+1;
                Road(i).lane(lane_i).sample_x(count_i) = lane(lane_i).StartPoint(1)+ sample_length * cos(Road(i).geometry.hdg);                                       
                Road(i).lane(lane_i).sample_y(count_i) = lane(lane_i).StartPoint(2)+ sample_length * sin(Road(i).geometry.hdg);
            end            
        end
    elseif Road(i).geometry.type == 3
        
        count_i=0;
        for sample_length= 0:sample_factor:Road(i).geometry.length
            count_i=count_i+1;
            
            SampleAlpha = sample_length  / ( Road(i).geometry.Curve(1) * 2) +  Road(i).geometry.hdg;
            for lane_i=1:length(Road(i).lane)
                if Road(i).lane(lane_i).id == 0
                    lane(lane_i).StartPoint = Road(i).geometry.SPoint;
                    Curve_r(lane_i)=Road(i).geometry.Curve(1);
                else
                    width=Road(i).lane(lane_i).width;                    
                    lane(lane_i).StartPoint = [lane(lane_i-1).StartPoint(1)+ width*cos(Road(i).geometry.hdg-pi/2) lane(lane_i-1).StartPoint(2)+ width*sin(Road(i).geometry.hdg-pi/2)];
                    Curve_r(lane_i) = Curve_r(lane_i-1)+ Road(i).lane(lane_i).width;
                end                
                Sample_a(lane_i) = 2 * Curve_r(lane_i) * sin (sample_length / ( Road(i).geometry.Curve(1) * 2));
                Road(i).lane(lane_i).sample_x(count_i) = lane(lane_i).StartPoint(1) + Sample_a(lane_i) * cos(SampleAlpha);
                Road(i).lane(lane_i).sample_y(count_i) = lane(lane_i).StartPoint(2) + Sample_a(lane_i) * sin(SampleAlpha);
            end
            
        end  
        
    end      
end


end

