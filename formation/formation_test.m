% 1. 相对位置式队形定义；2. 队形定义方法简单。
% problems: 1.没有避障； 2.没有避免碰撞。3. 运行过程无法随意进出队形

clear;
clc;
av_min_dis = 0.1;
a = 3^(0.5);
target = [
2 0;
1 a;
-1 a;
-2 0;
-1 -a;
1 0];
target_size = size(target,1);

gama_star = zeros(2, target_size, target_size);

for i = 1:target_size
    gama_star(:,:,i) =  repmat(target(i,:)', [1,target_size]) - target';
end

initial_pos = target.*-0.2 + repmat([3], size(target));
initial_pos = randn(size(target)) + repmat([3], size(target));
initial_vel = target.*(0);
vel = initial_vel;
pos = initial_pos;

A =[
    0 1 0 0 0 1;
    1 0 1 0 0 0;
    0 1 0 0 1 1;
    0 0 0 0 0 1;
    0 0 1 0 0 0;
    1 0 1 1 0 0]; % connectivity of the Nodes

dt = 0.01; %s 
alpha = 1;
beta = 1;
gama = zeros(2, target_size, target_size);
gama_diff = zeros(target_size,2);

u_save = zeros(0,target_size,2);
vel_save = zeros(0,target_size,2);

pos_save = zeros(0,target_size,2);

for index_t = (0:dt:20)
    
    for i = 1:target_size
        gama(:,:,i) =  repmat(pos(i,:)', [1,target_size]) - pos';
        gama_diff(i,:) = A(i,:)*(gama(:,:,i) - gama_star(:,:,i))';
    end 
    P_p = boundary(target, pos);
    
    items_v = vel.*alpha
    items_e = gama_diff;
    items_p = (pos-P_p).*beta;
%     items_p = 0;
    
    u = - items_v  - items_e  - items_p
    [pos,vel] = dynamics(u, vel, pos, dt);
    
%     u_save(end+1,:,:) = u; 
%     vel_save(end+1,:,:) = vel; 
    pos_save(end+1,:,:) = pos; 
    
    error = sum(items_e.^2,2).^0.5;
    some_error = av_min_dis<error;
    if  sum(some_error) > 0 
        disp('some_error: '); disp(error);
    else
        break;
    end
end

aplha=0:pi/40:2*pi;
r=2.4;
x=r*cos(aplha);
y=r*sin(aplha);
plot(x,y,'-');
hold on;
scatter(initial_pos(:,1), initial_pos(:,2),50,'cs');
plot(target(:,1), target(:,2),'r*-');
plot(pos_save(end,:,1),pos_save(end,:,2),'ks--');
axis equal;
pause(1);

% for i = 1:target_size
%     scatter(pos_save(:,i,1), pos_save(:,i,2),10,'r.');
% end

for tt = 1:5:size(pos_save,1)
    for i = 1:target_size
        scatter(pos_save(tt,i,1), pos_save(tt,i,2),10,'bo');
        pause(0.0002);
    end
end

disp('all done');



  