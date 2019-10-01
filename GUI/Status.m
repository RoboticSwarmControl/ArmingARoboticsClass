    %Display update
    function Status(Display,message,varargin)
        index = size(Display.String,1);
        if nargin == 1
            Display.String = '';
        elseif index && nargin < 3
            previous = Display.String(index,:);
            if contains(previous,message)
                if index == 1
                    Display.String = strcat(Display.String,'.');
                else
                    Display.String(index,:) = strcat(Display.String(index,:),'.');
                end
            elseif index < 7
                Display.String = [Display.String;{message}];
            else
                Display.String(1,:) = [];
                Display.String = [Display.String;{message}];
            end
        else
            Display.String = message;
        end        
    end