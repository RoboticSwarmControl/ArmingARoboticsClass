function Update(Data)
    %UPDATE THE FIGURE WITH THE SLIDE VALUES    
    for i = 1:size(Data.input,1) %for every arm but the gripper, update the transforms
        DH(i,:) = [Data.input(i,1), Data.input(i,2), Data.input(i,3), -Data.Slide(i).Value];
        if i == 1
            Data.HGTrans(i).Matrix  = Htrans(DH(i,:));
        else
            Data.HGTrans(i).Matrix  = Data.HGTrans(i-1).Matrix * Htrans(DH(i,:));
        end
    end
    DH(:,4) = -DH(:,4).*RadCheck(1:size(Data.input,1),Data.Slide,Data.Radians)';
    Data.DHTable.Data = flipud(DH);
    %gripper transforms
    %pi/90 constant converts a full scale value of 45mm to a pi/2 degree turn
    Data.GripTrans(1).Matrix = Data.HGTrans(4).Matrix * Htrans([   0, pi/2,  12.1, 0]) * Htrans([0,0,0, Data.Slide(5).Value * pi/90]);
    Data.GripTrans(2).Matrix = Data.HGTrans(4).Matrix * Htrans([   0, pi/2, -12.1, 0]) * Htrans([0,0,0,-Data.Slide(5).Value * pi/90]);
    Data.GripTrans(3).Matrix = Data.HGTrans(4).Matrix * Htrans([16.8, pi/2,   6.3, 0]) * Htrans([0,0,0, Data.Slide(5).Value * pi/90]);
    Data.GripTrans(4).Matrix = Data.HGTrans(4).Matrix * Htrans([16.8, pi/2,  -6.3, 0]) * Htrans([0,0,0,-Data.Slide(5).Value * pi/90]);
    Data.GripTrans(5).Matrix = Data.GripTrans(3).Matrix * Htrans([  23,    0,     0, 0]) * Htrans([0,0,0,-Data.Slide(5).Value * pi/90]);
    Data.GripTrans(6).Matrix = Data.GripTrans(4).Matrix * Htrans([  23,    0,     0, 0]) * Htrans([0,0,0, Data.Slide(5).Value * pi/90]);
end
%{
    RadCheck
    Htrans
%}
