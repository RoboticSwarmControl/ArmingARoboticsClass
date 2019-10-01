function result = ImproperPinValues(Display,Apin,En,Dir)
    % CHECK FOR LEGAL PIN VALUES
    result = 1;    
    % check for empty and non integer entries
    if EmptyEntries(Apin) 
        Status(Display,'Please Check analog Pins for omitted inputs,');
        Status(Display,'and for non-integer inputs');
    elseif EmptyEntries(En)
        Status(Display,'Please Check digital Pins for omitted inputs,');
        Status(Display,'and for non-integer inputs');
    elseif EmptyEntries(Dir)
        Status(Display,'Please Check Direction Pins for omitted inputs,');
        Status(Display,'and non-integer inputs');
    else
        for i = 1:numel(Apin)
            APins(i) = eval(Apin{i});
        end
        for i = 1:numel(En)
            EPins(i) = eval(En{i});
        end
        for i = 1:numel(Dir)
            DPins(i) = eval(Dir{i});
        end
        EPins(6) = eval(En{6});
        % make sure pins are not repeated
        if RepeatedEntries(APins)
            Status(Display,'Repeated analog input pin');
        elseif RepeatedEntries([EPins,DPins])
            Status(Display,'Repeated digital output pin');
        % make sure pins are within the right range
        elseif OutofRange(APins,'A')
            Status(Display,'An analog input pin is out of range');
        elseif OutofRange([EPins,DPins],'D')
            Status(Display,'A Digital output pin is out of range');
        else
            result = 0;
        end
    end
end
%{
    EmptyEntries
    Status
    OutofRange
    RepeatedEntries
%}