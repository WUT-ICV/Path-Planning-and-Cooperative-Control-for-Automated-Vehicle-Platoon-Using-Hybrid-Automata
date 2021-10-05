function CarState = CarLocation(RoadLocation,Road)
% RoadLocation = [RoadNum, LaneID, s]
global aTestCounter_DisToLaneFunction;
aTestCounter_DisToLaneFunction=aTestCounter_DisToLaneFunction+1;
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
RoadNum = RoadLocation(1);
LaneID = RoadLocation(2);
s = RoadLocation(3)/Road(RoadNum).SampleFactor;

CarState.v = RoadLocation(4);
CarState.wheelang = RoadLocation(5);
for lane_i = 1:length(Road(RoadNum).lane)
    if Road(RoadNum).lane(lane_i).id == LaneID
        CarState.x = (Road(RoadNum).lane(lane_i-1).sample_x(round(s)+1)+Road(RoadNum).lane(lane_i).sample_x(round(s)+1))/2;
        CarState.y = (Road(RoadNum).lane(lane_i-1).sample_y(round(s)+1)+Road(RoadNum).lane(lane_i).sample_y(round(s)+1))/2;
        CarState.phi = Road(RoadNum).samplepoint.hdg(round(s)+1);
    end         
end

end

