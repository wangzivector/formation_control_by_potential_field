function [pos_f,vel_f] = dynamics(acc, vel, pos, dt)
    dd = 100;
    ddt = dt / dd;
    pos_f = zeros(size(pos));
    vel_f=  zeros(size(vel));
    for index = 1:size(pos,1)
        acc_i = repmat(acc(index,:), [dd, 1]);
        vel_i = repmat(vel(index,:), [dd, 1]);
        pos_i= repmat(pos(index,:), [dd, 1]);

        for i =  2:dd
            acc_i(i,:) = acc_i(i-1,:);
            pos_i(i,:) = pos_i(i-1,:)+ vel_i(i-1,:).*ddt + acc_i(i-1,:).*(0.5*ddt^2);
            vel_i(i,:) = vel_i(i-1,:) + acc_i(i-1,:).*ddt;
        end
        pos_f(index,:) = pos_i(dd,:);
        vel_f(index,:) =vel_i(dd,:);
    end
end