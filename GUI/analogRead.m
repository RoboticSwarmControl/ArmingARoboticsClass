function result = analogRead(Arduino,pin)
    %ANALOG READ
    result = zeros(size(pin));
    pin = strcat('A',num2str(pin));
    for i = 1:size(pin,1)
        result(i) = Arduino.readVoltage(pin(i,:));
    end
end
