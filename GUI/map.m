function result = map(val,range1,range2)
    %MAP FUNCTION (converts a val in range1 to a result in range2)
    x1 = range1(:,1);
    y1 = range1(:,2);
    x2 = range2(:,1);
    y2 = range2(:,2);
    result = (val - x1).*(y2 - x2)./(y1-x1)+ x2;
end
