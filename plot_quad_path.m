%% Gather Basic Plot Info

close all;

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
    ylim([-2,2])
    grid on
    xlabel('time [s]')
    ylabel(labels{i})
end

legend('Actual','Commanded', 'Location', 'best');
subplot(3,1,1);
title('X,Y,Z Position (Trajectory 4)');

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

legend('Actual','Commanded', 'Location', 'best');
subplot(3,1,1);
title('X,Y,Z dot (Trajectory 4)');


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

legend('Actual','Commanded', 'Location', 'best');
subplot(3,1,1);
title('Roll, Pitch, Yaw (Trajectory 4)');

%% Plot path in 3D

figure;
hold on;
labels = {'x [m]', 'y [m]', 'z [m]'};

% Plot Path
plot3(ViconData(1+3,start:finish),ViconData(2+3,start:finish),ViconData(3+3,start:finish), line_color{1}, 'LineWidth', line_width);
plot3(pos_des(:,1), pos_des(:,2), pos_des(:,3), line_color{2}, 'LineWidth', line_width);


% Set up Figure

xlim([-2,2]);
ylim([-2,2]);
zlim([-2,2]);
grid on;
axis equal;
xlabel(labels{1})
ylabel(labels{2})
zlabel(labels{3})

legend('Actual','Commanded', 'Location', 'best');
title('3D Position (Trajectory 4)');

