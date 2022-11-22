file =  fopen('texto.txt');

content = textscan(file, '%f %f %f')
content = cell2mat(content)

target_speed = content(:,1);
speed = content(:,2);
distance = content(:, 3);

figure
plot(distance, target_speed, distance, speed )