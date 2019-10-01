function digitalWrite(Arduino,pin,Value)
    %DIGITAL WRITE
    for i = 1:numel(pin)
        Arduino.writeDigitalPin(strcat('D',num2str(pin(i))),Value(i));
    end
end
