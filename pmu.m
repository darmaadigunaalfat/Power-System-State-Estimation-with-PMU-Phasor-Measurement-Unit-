% Power System State Estimation with Synchronized Phasor Measurements..
% Praviraj PG, Oct-2009, IIT Roorkee

function E3 = pmu(num,E2,W1)

zdatap = zdataps(num); % Get Phasor Measurement data..
type = zdatap(:,2); % Type of measurement, Vi - 1, Pi - 2, Qi - 3, Pij - 4, Qij - 5, Iij - 6..
magn = zdatap(:,3); % Measuement values..
ang = zdatap(:,4); % Angles..
fbus = zdatap(:,5); % From bus..
tbus = zdatap(:,6); % To bus..
Rim = zdatap(:,7); % Measurement Error..
Ria = zdatap(:,8);
Y = ybusppg(num);
bpq = bbusppg(num);
G = real(Y);
B = imag(Y);
nbus = length(Y);

vi = find(type == 1); % Index of measurements..
ii = find(type == 2);

nvi = length(vi); % Number of Voltage measurements..
nii = length(ii); % Number of Real Power Injection measurements..

V = E2(:,1);    Del = E2(:,2);  del  = (pi/180)*Del;   % Output from WLS..
Esr = V.*cos(del);
Esi = V.*sin(del);
Vpr = magn(1:nvi).*cos(ang(1:nvi));
Vpi = magn(1:nvi).*sin(ang(1:nvi));
Ipr = magn(nvi+1:end).*cos(ang(nvi+1:end));
Ipi = magn(nvi+1:end).*sin(ang(nvi+1:end));
M = [Esr; Esi; Vpr; Vpi; Ipr; Ipi]; % Measurement Vector..

% Forming new Jacobian Matrix
J11 = eye(nbus,nbus);
J12 = zeros(nbus,nbus);    
J21 = J12;
J22 = J11;
    
J31 = zeros(nvi,nbus);
for i = 1:nvi
    m = fbus(vi(i));
    for k = 1:nbus
        if k == m
           J31(i,k) = 1;
        end
    end
end
    
J32 = zeros(nvi,nbus);    
J41 = J32;    
J42 = J31;   

J51 = zeros(nii,nbus);
for i = 1:nii
    m = fbus(ii(i));
    n = tbus(ii(i));
    for k = 1:nbus
        if k == m
           J51(i,k) = -G(m,n);
        else if k == n
                J51(i,k) = G(m,n);
             end
        end
    end
end
    
J52 = zeros(nii,nbus);
for i = 1:nii
    m = fbus(ii(i));
    n = tbus(ii(i));
    for k = 1:nbus
        if k == m
           J52(i,k) = B(m,n)-bpq(m);
        else if k == n
                J52(i,k) = -B(m,n);
             end
        end
    end
end
    
J61 = -J52;   
J62 = J51;
    
% New Measurement Jacobian, Hc..
Hc = [J11 J12; J21 J22; J31 J32; J41 J42; J51 J52; J61 J62];

% Rotation Matrix..
R11 = diag(cos(ang(1:nvi)));
R12 = diag(-magn(1:nvi).*sin(ang(1:nvi)));
R21 = diag(sin(ang(1:nvi)));
R22 = diag(magn(1:nvi).*cos(ang(1:nvi)));
R31 = diag(cos(ang(nvi+1:end)));
R32 = diag(-magn(nvi+1:end).*sin(ang(nvi+1:end)));
R41 = diag(sin(ang(nvi+1:end)));
R42 = diag(magn(nvi+1:end).*cos(ang(nvi+1:end)));
Rv = [R11 R12; R21 R22];        % For Voltage phasors..
Ri = [R31 R32; R41 R42];        % For Current phasors..

Wv = [Rim(1:nvi); Ria(1:nvi)];
Wi = [Rim(nvi+1:end); Ria(nvi+1:end)];

W2 = [Rv.^2*Wv; Ri.^2*Wi];

W = diag([W1; W2]);

R = diag(1./W);
R = diag(R);    % New Covariance Matrix..

% State Vector..
E = inv(Hc'*R*Hc)*(Hc'*R)*M; % In Rectangular cordinates..
[del Vm] = cart2pol(E(1:nbus),E(nbus+1:end)); % In Polar Cordinates..
Del = 180/pi*del;
V = Vm;
E3 = [V Del]; % Bus Voltages and angles..
disp('------------- State Estimation with PMUs ----------------------------------------------');
disp('--------------------------');
disp('| Bus |    V   |  Angle  | ');
disp('| No  |   pu   |  Degree | ');
disp('--------------------------');
for m = 1:n
    fprintf('%4g', m); fprintf('  %8.4f', V(m)); fprintf('   %8.4f', Del(m)); fprintf('\n');
end
disp('---------------------------------------------------------------------------------------');