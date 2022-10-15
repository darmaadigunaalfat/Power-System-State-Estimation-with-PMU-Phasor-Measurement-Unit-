% Power System State Estimation with PMUs (Phasor Measurement Units)..
% Darma Adi Guna Alfat, Tugas UTS
% This is the main program..

num = 14;                   % Bus System..IEEE-30 or IEEE-14..
E1 = nrlfppg(num);          % Obtain State from NRLF..
[E2, W1] = wls(num);        % Obtain State from WLS..
E3 = pmu(num,E2,W1);        % Estimated State from WLS+PMU..
errors(E1,E2,E3);           % Calculate Error w.r.t NRLF and Plot Errors..