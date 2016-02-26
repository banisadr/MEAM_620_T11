%% Gather Basic Plot Info

features = find(diff(DesPosSave(1,:)));
start = features(2);
[c, finish] = min(abs(timer-timer(start)-t));
%finish = features(3);
tot_time = timer(finish)-timer(start);
t_off = timer(start);
times(end) = t;
line_color = {'r', 'b'};
line_width = 1;

%% Plot x,y,z position
labels = {'x [m]', 'y [m]', 'z [m]'};

% for i = 1:3
%     subplot(3, 1, i)
%     hold on
%     plot(timer(start:finish)-t_off, DesPosSave(i,start:finish), 'g', 'LineWidth', line_width);
%     hold off
%     xlim([timer(start), timer(finish)]-t_off)
%     grid on
%     xlabel('time [s]')
%     ylabel(labels{i})
% end

for i = 1:3
    subplot(3, 1, i)
    hold on
    plot(timer(start:finish)-t_off, ViconData(i+3,start:finish), line_color{1}, 'LineWidth', line_width);
    plot(times, pos_des(:,i), line_color{2}, 'LineWidth', line_width);
    hold off
    xlim([timer(start), timer(finish)]-t_off)
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
end

legend('Actual','Desired', 'Location', 'best');
subplot(3,1,1);
title('X,Y,Z Position');

%% Plot x,y,z velocity

figure
labels = {'x [m/s]', 'y [m/s]', 'z [m/s]'};

for i = 1:3
    subplot(3, 1, i)
    hold on
    plot(timer(start:finish)-t_off, VelSave(i,start:finish), line_color{1}, 'LineWidth', line_width);
    plot(times, vel_des(:,i), line_color{2}, 'LineWidth', line_width);
    hold off
    xlim([timer(start), timer(finish)]-t_off)
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
end

legend('Actual','Desired', 'Location', 'best');
subplot(3,1,1);
title('X,Y,Z dot');


%% Plot roll, pitch, yaw

figure
labels = {'roll [rad]', 'pitch [rad]', 'yaw [rad]'};

for i = 1:3
    subplot(3, 1, i)
    hold on
    plot(timer(start:finish)-t_off, ViconData(i,start:finish), line_color{1}, 'LineWidth', line_width);
    plot(timer(start:finish)-t_off, trpySave(i+1,start:finish), line_color{2}, 'LineWidth', line_width);
    hold off
    xlim([timer(start), timer(finish)]-t_off)
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
end

legend('Actual','Desired', 'Location', 'best');
subplot(3,1,1);
title('Roll, Pitch, Yaw');
