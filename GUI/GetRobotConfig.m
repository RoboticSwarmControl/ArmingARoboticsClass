function result = GetRobotConfig(Arduino,Pins,Limits,Slides,Rad)
    %RETURNS THE PHYSICAL ARM'S CURRENT CONFIG TO THE PROGRAM
    radresult = map(analogRead(Arduino,Pins.Ana'),[[Pins.Pot.Min.Value]' [Pins.Pot.Max.Value]'],Limits);
    result = radresult.*RadCheck(1:5,Slides,Rad)';
end
%{
    map
    RadCheck
%}
