function [Carhost,Num_CoCar,CoCar,Num_obCar,obCar]=NewRelation(Car,HostID)

    Carhost=Car(HostID);    
    %分出环境车辆状态
    obCar=Car;
    obCar(HostID)=[];       
    %分出可通信车辆
    row_index = cat(1,obCar.comEn) == Carhost.comEn;
    Num_CoCar=sum(row_index);
    CoCar=obCar(row_index);
    %分出不可通信障碍车辆
    [m,n]=size(Car);
    Num_obCar=n-Num_CoCar-1;
    obCar(row_index)=[];
    
end