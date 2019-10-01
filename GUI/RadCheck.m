function result = RadCheck(index,Slides,Rad)
    %CONVERTS FROM RAD TO DEGRESS IF RAD BUTTON IS PUSHED
    result = ones(size(index));
    if ~Rad.Value
        result([Slides(index).String]=='R') = 180/pi;
    end
end
