function [DisToEachLane,Roadhdg,Roadwidth,DisToCenter,RoadNum,s,LocationLaneID] = testDisToLaneFunction( x,y,Road)

global aTestCounter_DisToLaneFunction;
aTestCounter_DisToLaneFunction=aTestCounter_DisToLaneFunction+1;

[temp_m,Num_Section] = size(Road);
MinDis.NumRoad = 0;
MinDis.Dis = -1;
MinDis.location = 0;


for i=1:Num_Section
    DisToRoad = (x-Road(i).samplepoint.x(1:100)).^2 + (y-Road(i).samplepoint.y(1:100)).^2;
    [Dis,location]=min(DisToRoad);
    if Dis < MinDis.Dis || MinDis.Dis == -1
        MinDis.Dis = Dis;
        MinDis.location = location;
        MinDis.NumRoad = i;
    end   
end

DisToEachLane = x;
Roadhdg = y;
Roadwidth = Road(1).width;
DisToCenter = 0;
RoadNum = 0;
s=0;
LocationLaneID=0;
end

