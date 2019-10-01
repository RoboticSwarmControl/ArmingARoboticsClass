function result = GetPlotConfig(Slides,Rad)
    %RETURNS THE VIRTUAL ARM'S CONFIG
    result = [Slides.Value].*RadCheck(1:5,Slides,Rad);
end
%{
    RadCheck
%}