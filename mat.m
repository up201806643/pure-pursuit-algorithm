file =  fopen('texto.txt');

content = textscan(file, '%f %f %f')
content = cell2mat(content)

target_speed = content(:,1);
speed = content(:,2);
distance = content(:, 3);

figure
plot(distance, target_speed )
hold on
plot(distance, speed)

% f = fopen('target_speed.txt', 'w');
% 
% for v = 1:1:2100
%    x(v) = v;
% end
% 
% format = '%d\n';
% fprintf(f,format, x);