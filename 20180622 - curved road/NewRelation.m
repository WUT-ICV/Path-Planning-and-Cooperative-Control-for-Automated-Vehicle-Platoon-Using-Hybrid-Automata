function [Carhost,Num_CoCar,CoCar,Num_obCar,obCar]=NewRelation(Car,HostID)

    Carhost=Car(HostID);    
    %�ֳ���������״̬
    obCar=Car;
    obCar(HostID)=[];       
    %�ֳ���ͨ�ų���
    row_index = cat(1,obCar.comEn) == Carhost.comEn;
    Num_CoCar=sum(row_index);
    CoCar=obCar(row_index);
    %�ֳ�����ͨ���ϰ�����
    [m,n]=size(Car);
    Num_obCar=n-Num_CoCar-1;
    obCar(row_index)=[];
    
end