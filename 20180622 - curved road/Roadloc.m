function milestone = Roadloc( Car,Road )
%UNTITLED15 Summary of this function goes here
%   Detailed explanation goes here
milestone = Car.RoadLocation(3) ;
if Car.RoadLocation(1) > 1
    for Road_i = 1:Car.RoadLocation(1)-1
        milestone = milestone + Road(Road_i).geometry.length;
    end

end

