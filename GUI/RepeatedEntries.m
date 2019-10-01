function result = RepeatedEntries(Pins)
    %CHECK IF ANY DIGITAL OR ANALONG PINS ARE REPEATED    
    result = 0;
    for i = 1:numel(Pins)-1
        checkval = Pins(i);
        for k = i:numel(Pins)-1
            if checkval == Pins(k+1)
                result = 1;
                break;
            end
        end
        if result
            break;
        end
    end
end
