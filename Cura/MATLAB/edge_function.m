% If you want excute this, current folder must be correct
clear all;
close all;
upupfolder = fileparts(fileparts(pwd));
file_path = fullfile(upupfolder,'moveo_moveit_move_group','area','outermost_area.csv');
File = csvread(file_path);% read outermost area file
[num1,num2] = size(File);
X_Plane = File(:,[2,3]);% take y and z
Y_position = X_Plane(:,1)'.*100;% take y
Z_position = X_Plane(:,2)'.*100;% take z
Shift_Y_position = Y_position-1;% shift y, that we have safe area
plot(Z_position,Y_position,'o');
grid on;
hold on;
Edge_function = polyfit(Z_position,Shift_Y_position,10);
x1 = 0:0.0001:((num1-1)/10);
y1 = polyval(Edge_function,x1)';
plot(x1,y1,'r');
output_file_path = fullfile(upupfolder,'Cura','edge_function','edge_function.txt');
Fid=fopen(output_file_path,'w');
fprintf(Fid,'%32.30f\n',Edge_function);
fclose(Fid);
clear all;