function result = CheckCal(SetArray)
    % CHECK IF ALL PINS HAVE BEEN CALIBRATED
    if  ([SetArray.Min(1:5).Set] & [SetArray.Max(1:5).Set])
        result = 1;
    else
        result = 0;
    end
end
