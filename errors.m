% State Estimation Errors..
% Praviraj PG, Oct-2009, IIT Roorkee

function errors(E1,E2,E3)

er11 = E1-E2;
er12 = E1-E3;
n = length(er11);
t = 1:n;

subplot(2,2,1);
plot(t,er11(:,2),'--rs','LineWidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
set(gca,'XTick',1:1:n);
title('Voltage Angle Estimation Error without PMU');
xlabel('Bus Number'); ylabel('Voltage Angle Error (degrees)');

subplot(2,2,2);
plot(t,er11(:,1)*100,'--rs','LineWidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
set(gca,'XTick',1:1:n);
title('Voltage Magnitude Estimation Error without PMU');
xlabel('Bus Number'); ylabel('Voltage Magnitude Error (%)');

subplot(2,2,3);
plot(t,er12(:,2),'--rs','LineWidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
set(gca,'XTick',1:1:n);
title('Voltage Angle Estimation Error with PMU');
xlabel('Bus Number'); ylabel('Voltage Angle Error (degrees)');

subplot(2,2,4);
plot(t,er12(:,1)*100,'--rs','LineWidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
set(gca,'XTick',1:1:n);
title('Voltage Magnitude Estimation Error with PMU');
xlabel('Bus Number'); ylabel('Voltage Magnitude Error (%)');