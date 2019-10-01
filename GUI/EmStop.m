function EmStop(Arduino,Turn,Pins,val)
    %EMERGENCY STOP
    digitalWrite(Arduino,Pins(val), ~Turn*ones(size(val)));
    digitalWrite(Arduino,Pins(val), ~Turn*ones(size(val)));
end
