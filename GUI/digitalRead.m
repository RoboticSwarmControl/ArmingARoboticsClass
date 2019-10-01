function result = digitalRead(Arduino,pin)
    %ANALOG READ
    result = zeros(size(pin));
    pin = strcat('D',num2str(pin));
    for i = 1:size(pin,1)
        result(i) = Arduino.readDigitalPin(pin(i,:));
    end
end
