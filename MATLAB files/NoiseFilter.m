function output = NoiseFilter(input)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    j = 1;
    for i = 1:length(input)
        if length(input{i,1}) > 200
            output{j,1} = input{i,1};
            j = j+1;
        end
    end
end

