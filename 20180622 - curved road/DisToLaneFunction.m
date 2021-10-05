function [DisToEachLane,Roadhdg,Roadwidth,DisToCenter,RoadNum,s,LocationLaneID] = DisToLaneFunction( x,y,Road)

global aTestCounter_DisToLaneFunction;
aTestCounter_DisToLaneFunction=aTestCounter_DisToLaneFunction+1;

if aTestCounter_DisToLaneFunction == 14817
    aTestCounter_DisToLaneFunction=aTestCounter_DisToLaneFunction;
end
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
[temp_m,Num_Section] = size(Road);
MinDis.NumRoad = 0;
MinDis.Dis = -1;
MinDis.location = 0;


for i=1:Num_Section
    DisToRoad = (x-Road(i).samplepoint.x).^2 + (y-Road(i).samplepoint.y).^2;
    [Dis,location]=min(DisToRoad);
    if Dis < MinDis.Dis || MinDis.Dis == -1
        MinDis.Dis = Dis;
        MinDis.location = location;
        MinDis.NumRoad = i;
    end   
end
if MinDis.Dis~=-1
    MinDis.Dis=sqrt(MinDis.Dis);
end
for lane_i = 1:length(Road(i).lane)
    minx=Road(MinDis.NumRoad).lane(lane_i).sample_x(MinDis.location);
    miny=Road(MinDis.NumRoad).lane(lane_i).sample_y(MinDis.location);
    DisToEachLane(lane_i) = sqrt((x-minx)^2 + (y-miny)^2);
end
RoadCenter.x = (Road(MinDis.NumRoad).lane(1).sample_x(MinDis.location)+Road(MinDis.NumRoad).lane(length(Road(i).lane)).sample_x(MinDis.location))/2;
RoadCenter.y = (Road(MinDis.NumRoad).lane(1).sample_y(MinDis.location)+Road(MinDis.NumRoad).lane(length(Road(i).lane)).sample_y(MinDis.location))/2;
DisToCenter = sqrt((x-RoadCenter.x)^2 + (y-RoadCenter.y)^2);
Roadwidth=Road(MinDis.NumRoad).width;
Roadhdg=Road(MinDis.NumRoad).samplepoint.hdg(MinDis.location);
RoadNum=MinDis.NumRoad;
s=Road(MinDis.NumRoad).SampleFactor * MinDis.location;
flag=0;
for lane_i=2:length(Road(i).lane)
    if DisToEachLane(lane_i)<=Road(MinDis.NumRoad).lane(lane_i).width && DisToEachLane(lane_i-1)<=Road(MinDis.NumRoad).lane(lane_i).width
        LocationLaneID = Road(MinDis.NumRoad).lane(lane_i).id;
        flag=1;
    end
end   
if flag==0
    LocationLaneID = 404;
end
end

