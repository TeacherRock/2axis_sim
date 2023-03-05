function Output = derivate(f, order, sampT, axis)

switch order
    case 1
        df = [zeros(1, axis); (f(3 : end, :) - f(1 : end-2, :))./(2*sampT); zeros(1, axis)];
        Output = df; 
    case 2
        ddf = [zeros(1, axis); (f(3 : end, :) -2*f(2 : end-1, :) + f(1 : end-2, :))/sampT^2; zeros(1, axis)];
        Output = ddf;
    case 3
        dddf = [zeros(2, axis);  (f(5 : end, :) - f(4 : end-1, :) + 2*f(2 : end-3, :) - f(1 : end-4, :))/(2*sampT^3); zeros(2, axis)];
        Output = dddf; 
    case 4
        ddddf = [zeros(2, axis);  (f(5 : end, :) - 4*f(4 : end-1, :) + 6*f(3 : end-2, :) - 4*f(2 : end-3, :) + f(1 : end-4, :))/sampT^4; zeros(2, axis)];
        Output = ddddf; 

end

end
