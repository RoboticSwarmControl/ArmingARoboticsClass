function analogWrite(Arduino,pin,Value)
    %ANALOG WRITE
    for i = 1:numel(pin)
        Arduino.writePWMVoltage(strcat('D',num2str(pin(i))),Value(i));
    end
end
