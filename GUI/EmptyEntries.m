function result = EmptyEntries(CharArray)
    result = 0;
    for i = 1:numel(CharArray)
        value = str2double(CharArray(i));
        if isempty(CharArray(i))
            result = 1;
            break;
        elseif isnan(value)||mod(value,1)~=0
            result = 2;
            break;
        end
    end
end
