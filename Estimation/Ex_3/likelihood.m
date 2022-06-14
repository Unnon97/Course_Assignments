function [outputArg] = likelihood(snrSamples,range)
    % Task 2
    meanSnr = measurementFunction(range);
    outputArg = (1/meanSnr)*exp(-snrSamples/meanSnr);
end

