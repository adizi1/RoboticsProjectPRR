function PRR_simulation(P_s,P_e,alpha) % this is the main function
    % P_s [cm] is the initial position of the tool in Cartesian coordinates and P_e [cm]
    % is the end position of the tool at the end of the movement
    %% Definitions of the robotic system form
    l1=10*0.01;%[m]
    l2=3*0.01;%[m]
    l3=3*0.01;%[m]
    Kp=5;% Controller gains
    Kd=3;
    % links size, mass,
    m2=3; %[kg]
    m3=5; %[kg]
    h=1*0.01; %[m]
    w=1*0.01;
    tau = 1;
    customPurple = [0.4, 0, 0.6]; % Deep violet
    customOrange = [1, 0.5, 0];
    %% find the joints angales for the desired trajectory
    xs=P_s(1)*0.01; ys=P_s(2)*0.01; zs=P_s(3)*0.01; % starting point in [m]
    xe=P_e(1)*0.01; ye=P_e(2)*0.01; ze=P_e(3)*0.01;% ending point in [m]
    [D,xdydzd]=minimal_jerk(xs,ys,zs,xe,ye,ze,tau,l1,l2,l3);

    function [desiredTrajectory,xdydzd]=minimal_jerk(xs,ys,zs,xe,ye,ze,tau,l1,l2,l3)
         % calculate the desierd trajectory according to minimal jerk :
         t = linspace(0, tau, 100); % Time vector with 100 time steps from 0 to tau
        
         % Preallocate space for trajectory, velocity, and acceleration
         q = zeros(length(t), 3);
         velocity = zeros(length(t), 3); % Joint velocities
         acceleration = zeros(length(t), 3); % Joint accelerations
         xdydzd = zeros(length(t), 3); % Joint locations matrix
        
         %calculate the joints angles and velocties and accelrations for 
         % this desired trajectory:
         for i = 1:length(t)
            % Minimal jerk trajectory parameter
            s = (t(i)/tau)^3 * (-10 + 15*(t(i)/tau) - 6*(t(i)/tau)^2);
            % Position of the end-effector
            xdydzd(i, :) = [xs + s*(xs-xe), ys + s*(ys-ye), zs + s*(zs-ze)];
            % Compute inverse kinematics for the current position
            [d1, t2, t3] = inverse_kinematics(xdydzd(i, 1), xdydzd(i, 2), xdydzd(i, 3), l1, l2, l3);
            % Store the joint positions
            q(i, :) = [d1, t2, t3];
         end
        
        % Calculate derivatives to obtain velocity and acceleration
        velocity(:, 1) = gradient(q(:, 1), t);
        velocity(:, 2) = gradient(q(:, 2), t);
        velocity(:, 3) = gradient(q(:, 3), t);
        acceleration(:, 1) = gradient(velocity(:, 1), t);
        acceleration(:, 2) = gradient(velocity(:, 2), t);
        acceleration(:, 3) = gradient(velocity(:, 3), t);
        
        % find the joints angales for the desired trajectory
        desiredTrajectory.time = t;
        desiredTrajectory.d1 = q(:, 1);
        desiredTrajectory.t2 = q(:, 2);
        desiredTrajectory.t3 = q(:, 3);
        desiredTrajectory.v1 = velocity(:, 1);
        desiredTrajectory.v2 = velocity(:, 2);
        desiredTrajectory.v3 = velocity(:, 3);
        desiredTrajectory.a1 = acceleration(:, 1);
        desiredTrajectory.a2 = acceleration(:, 2);
        desiredTrajectory.a3 = acceleration(:, 3);
        
     end
     % nestes inverse kinematics function to get joint angles
     function [d1, t2, t3] = inverse_kinematics(xe, ye, ze,l1,l2,l3)
         %distance from center
         r_squared = (l1-xe)^2 + ye^2;
    
        % check if the position is within the workspace of the robot
        if sqrt(r_squared) > l3+l2
            error('The position is out of reach.');
        end
        
        %find d1
        d1= ze;
        %the cosine sentence for theta 3
        %r^2=l2^2+l3^2-2l1*l2*cos(thetha 3)
        cos_theta3 = (r_squared - (l2^2) - (l3^2)) / (2 * l2 * l3); 
        %check if theta is real
        if cos_theta3< -1 || cos_theta3> 1
            error('The position is out of reach.');
        end
        %extract theta3
        t3 = acos(cos_theta3);  
        %sin^2+cos^2=1
        sin_theta3 = sqrt(1 - cos_theta3^2);
        %find the second solution for theta 2 [cos theta3 = cos -theta3]
        t2 = atan2(ye, xe-l1) - atan2(l3 * sin_theta3, l2 + l3 * cos_theta3);
    end
    
    %% plot the robot world- workspace and links
        % Define the workspace limits
    [d1, t2, t3] = inverse_kinematics(xe, ye, ze, l1, l2, l3);
   
    
    function [x2,y2,z2,x3,y3,z3]=plot_workspace(d1,t2,t3)
        % Plot the workspace of the robot, including links and joint
        legend_labels = {};
        x2 = l1+l2*cos(t2);
        y2 = l2*sin(t2);
        z2 = d1;
    
        x3 = x2 + l3*cos(t2 + t3);
        y3 = y2 + l3*sin(t2 + t3);
        z3 = d1;
    
        % drawing the links  of the robot in its current configuration
        %figure
        hold on;
        plot3([0 ,0], [0,0] ,[0,d1], 'k--' ,'LineWidth', 2); legend_labels{end+1} = 'Prismatic Joint';
        plot3([0, l1], [0, 0], [d1, d1], 'r-', 'LineWidth', 2); legend_labels{end+1} = 'Link';
        plot3([l1, x2], [0, y2], [d1, z2], 'r-', 'LineWidth', 2); legend_labels{end+1} = '';
        plot3([x2, x3], [y2, y3], [d1, z3], 'r-', 'LineWidth', 2); legend_labels{end+1} = '';
        hold on;

        % drawing the joints of the robot in its current configuration
        plot3(l1, 0, d1, 'ko', 'MarkerFaceColor', 'k');legend_labels{end+1} = 'Revolute Joint';
        plot3(x2, y2, z2, 'ko', 'MarkerFaceColor', 'k'); legend_labels{end+1} = '';
    
        [x_circle, y_circle]=xy_circle();
        plot3(x_circle, y_circle, d1 * ones(size(x_circle)), 'b--', 'LineWidth', 1.5);
        legend_labels{end+1} = 'Robot Workspace';
    
        % Label the axes
        xlabel('X [m]');
        ylabel('Y [m]');
        zlabel('Z [m]');
        title('PRR robot configuration 3D');
        grid on;
        axis equal;
        view(3); % 3D view
        hold on
    end
    
    function [x_circle, y_circle]=xy_circle()
        % Define the workspace limits
        % Plot the circular workspace boundary
        theta = linspace(0, 2*pi, 100); % Create a circle in XY plane
        x_circle = l1 + (l2 + l3) * cos(theta); %l2+l3 is the max radius for the workspace
        y_circle = (l2 + l3) * sin(theta);
    end
    function plot_desired()
        %this function plots the desired trajectory using the equasion
        xd = xdydzd(:,1);
        yd = xdydzd(:,2);
        zd = xdydzd(:,3);
        plot3(xd, yd, zd, 'Color', customPurple, 'LineWidth', 1.5);legend_labels{end+1} = 'Desired Trajectory';
        plot3(xd(1), yd(1),zd(1), '^', 'MarkerSize', 6, 'MarkerFaceColor',customOrange); legend_labels{end+1} = 'Starting Point';
        plot3(xd(end), yd(end),zd(end), '*', 'MarkerSize', 8, 'MarkerEdgeColor', customOrange); legend_labels{end+1} = 'End Point';
    end
    
    function plot_workspace2D()
        legend_labels = {};
        [x_circle, y_circle]=xy_circle();
        plot(x_circle, y_circle, 'b--', 'LineWidth', 1.5);legend_labels{end+1} = 'Robot Workspace';
        xy_circle();hold on
        plot([0, l1, x2, x3], [0, 0, y2, y3], '-r', 'LineWidth', 2);legend_labels{end+1} = 'Link';
        plot(l1, 0, 'ok', 'MarkerSize', 8, 'MarkerFaceColor', 'k');legend_labels{end+1} = 'Revolute Joint';
        plot(x2, y2, 'ok', 'MarkerSize', 8, 'MarkerFaceColor', 'k');legend_labels{end+1} = '';
        title('Robot Configuration in 2D');
        xlabel('X [m]');
        ylabel('Y [m]');
        grid on;
        axis equal;
        hold on;
    end

    % The workspace is based on the reach of the robot in the XY plane
    
    %% plots for the desired trajectory
    t = linspace(0, tau, 100);  
    figure()
    subplot(1,2,1)
    [x2,y2,~,x3,y3,~]=plot_workspace(d1,t2,t3);%plot the robot
    hold on
    plot_desired()%add desired trajectory
    legend(legend_labels,Location='best')
    hold on
    subplot(1,2,2)
    plot_workspace2D(); %plot work space with the desired trajectory
    plot_desired()
    legend(legend_labels,Location='best')
    % Desired trajectory 3D
    
    hold on;
    %% ode solver :
    initial_positions = [D.d1(1);D.t2(1);D.t3(1)];
    initial_velocities = [0;0;0];
    initial_torques =  [0;0;0];
    P0=[initial_positions, initial_velocities, initial_torques]; %initial condition
    t_s =0 ; % Initialization of the solved vector (x)
    t_e=1;
    %run on all time points using ode45
    %options = odeset('RelTol', 1e-3, 'AbsTol', [1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3]);
    [T,X] = ode45(@(t,x) springfunc(t,x,D,alpha, Kp, Kd), [t_s t_e], P0);%,options
    function dx = springfunc(t,x,D,alpha, Kp, Kd)
        %this is the nested function that defines the state-space model.
        % x(1) is position, x(2) is velocity and t is the time for the simulation solver
        % in this function you should create the dynamic equations of the robotic device
        %set inputs:
        l2=3*0.01;%[m]
        l3=3*0.01;%[m]
        g=9.81;%[N/m]
        d1 =x(1);
        t2=x(2); %positions theta2 theta3
        t3=x(3);
        %Define the sin cos angles
        s2=sin(t2);
        s3=sin(t3);  
        c2=cos(t2);  
        c3=cos(t3);    
        s23=sin(t2+t3);      
        c23=cos(t2+t3);
        
        %inertia tenzor
        Izz2=m2*(l2^2+w^2)/12;
        Izz3=m3*(l3^2+w^2)/12;
        
        %M,V,G
        M = [Izz2 + Izz3 + l2^2*(m2/4 +m3) + m3*l3*(l3/4 +l2*c3), Izz3+ m3*((l2^2)/4 + l2*l3*c3/2) ;
         Izz3+ m3*((l3^2)/4 + l3*l2*c3/2),  Izz3+(m3*(l3^2))/4];
        
        V = [-l2*l3*m3 *s3*x(5)*x(6) - (m3 *l2*l3*s3*(x(6))^2)/2;
              (m3*l2*l3*s3*(x(5))^2)/2];
        
        G = [-1.5*0.01*g*(m3*s23 + (m2 + 2*m3)*s2);
             -l3*g*m3*s23/2];
        % then dx is the first order equations for the solver (as you showed in q.1a in part 3)
        % variables that are defined in the main function are available here as well.
        % You can add other inputs for this function if needed
        d1_d = interp1(D.time, D.d1, t);
        t2_d = interp1(D.time, D.t2, t);
        t3_d = interp1(D.time, D.t3, t);
        d1_d_dot = interp1(D.time, D.v1, t);
        t2_d_dot = interp1(D.time, D.v2, t);%daccelaration
        t3_d_dot = interp1(D.time, D.v3, t);
        d1_d_ddot = interp1(D.time, D.a1, t);
        t2_d_ddot = interp1(D.time, D.a2, t);
        t3_d_ddot = interp1(D.time, D.a3, t);

        dx = zeros(9,1);
        %drag force
        c = 500; %the proportional constant
        Fdrag = c*3*0.01*[x(5)*(s2 + s23)+x(6)*s23 ; -x(5)*(c2 + c23)-x(6)*c23 ; 0];
        % jacobian
        J_L=[0 ,-3*0.01*(s2 + s23), -3*0.01*s23 ; 0, 3*0.01*(c2 + c23), 3*0.01*c23 ; 1, 0, 0];
        JL_Fdrag = J_L'*Fdrag;
        
        % torques of the joints with an addition of a proportional-derivative (PD)
        % controller with kp kd
        q_accelaration = M \( M * [t2_d_ddot; t3_d_ddot] - (1-alpha)*JL_Fdrag(2:3) - Kp *([t2; t3] - [t2_d; t3_d]) - Kd * ([x(5); x(6)] - [t2_d_dot; t3_d_dot])); 
        
        %differential equations
        dx(1) =0;% velocity =0, we have 2 DF
        dx(2) = x(5);
        dx(3) =x(6);
        dx(4) =0;% acceleration (from dynamics) =0
        dx(5)=q_accelaration(1);
        dx(6)=q_accelaration(2);
        % torques calculations:
        taus = M *[t2_d_ddot; t3_d_ddot] + V + G + alpha*JL_Fdrag(2:3)- Kp*([t2; t3] - [t2_d; t3_d]) ...
                      - Kd * ([x(4); x(5)] - [t2_d_dot; t3_d_dot]);
        dx(7)=0; %doesnt exist for a constant prismatic
        dx(8)=taus(1); %tau2
        dx(9)=taus(2); %tau3
    end
    
    %% forward kinemtics
    % nestes here you forward kinematics function
    function [T_mat]= DH2mat(DH_table)
        %this function gets the DH table (nX4) and returnes the  T forward kinematix matrix
        [n,~] = size(DH_table);
        
        %declarations
        T_mat = eye(4);%will be inserted with T values
        %T_temp = zeros(4);
        %creat temp t
        for i = 1:n
            %take variables out of the DH
            a_i1 =DH_table(i,1);
            alpha_i1=DH_table(i,2);
            d_i=DH_table(i,3);
            theta_i=DH_table(i,4);
        
            %for row i create matrix T from i-1 to i
            %row 1
            T_temp(1,1)=cos(theta_i);
            T_temp(1,2)=-sin(theta_i);
            T_temp(1,4)=a_i1;
            %row 2
            T_temp(2,1)=sin(theta_i)*cos(alpha_i1);
            T_temp(2,2)=cos(theta_i)*cos(alpha_i1);
            T_temp(2,3)=-sin(alpha_i1);
            T_temp(2,4)=-sin(alpha_i1)*d_i;
            %row 3
            T_temp(3,1)=sin(theta_i)*sin(alpha_i1);
            T_temp(3,2)=cos(theta_i)*sin(alpha_i1);
            T_temp(3,3)=cos(alpha_i1);
            T_temp(3,4)=cos(alpha_i1)*d_i;
            %row 4 all zeros except 4,4
            T_temp(4,4)=1;
            %multiply the matrices to update T_mat
            T_mat=T_mat*T_temp;
        end
        
    end
    %% Plotting the Desired and Real Trajectories
    % Initialize figure for plotting trajectories
    % plotting in 3D
    legend_labels = {};
 
    % Plot the Desired Trajectory
    desired_positions = zeros(3, length(D.time)); % Preallocate
    for v = 1:length(D.time)
        % Using DH2MAT to calculate end-effector position for each time step
        DH_table_desired = [0, 0, D.d1(v), 0;
                            l1, 0, 0, D.t2(v);
                            l2, 0, 0, D.t3(v);
                            l3, 0, 0, 0];
        T_desired = DH2mat(DH_table_desired);
        desired_positions(:, v) = T_desired(1:3, 4); % Extract end-effector position
    end
    figure;
    plot_workspace(D.d1(end),D.t2(end),D.t3(end));
    plot3(desired_positions(1, :), desired_positions(2, :), desired_positions(3, :), 'Color',customPurple, 'LineWidth', 2);
    legend_labels{end+1} = 'Desired Trajectory';
    % Plot the Real Trajectory
    real_positions = zeros(3, length(T)); % Preallocate
    for b = 1:length(T)
        % Using DH2MAT to calculate end-effector position for each time step
        DH_table_real = [0, 0, X(b, 1), 0; %real = X ( notation)
                         l1, 0, 0, X(b, 2);
                         l2, 0, 0, X(b, 3);
                         l3, 0, 0, 0];
        T_real = DH2mat(DH_table_real);
        real_positions(:, b) = T_real(1:3, 4); % Extract end-effector position
    end
    plot3(real_positions(1, :), real_positions(2, :), real_positions(3, :), 'g-', 'LineWidth', 2);legend_labels{end+1} = 'Real Trajectory';
    plot3(real_positions(1, 1), real_positions(2, 1), real_positions(3, 1), '^', 'MarkerSize', 6, 'MarkerFaceColor',customOrange); legend_labels{end+1} = 'Starting Point';
    plot3(real_positions(1,end), real_positions(2,end), real_positions(3,end), '*', 'MarkerSize', 6, 'MarkerEdgeColor', customOrange); legend_labels{end+1} = 'End Point'; 
    grid on; xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('3D Desired and Real Trajectories of the PRR Robot');
    hold on;
    % Add legend
    legend(legend_labels,Location='best');
    view(3); % Set view to 3D
    hold off;

    % plotting in 2D
    figure() %plot2D
    plot_workspace2D(); %plot work space 
    plot(desired_positions(1, :), desired_positions(2, :), 'Color',customPurple, 'LineWidth', 2);legend_labels{end+1} = 'Desired Trajectory';
    plot(real_positions(1, :), real_positions(2, :), 'g-', 'LineWidth', 2);legend_labels{end+1} = 'Real Trajectory';
    plot(real_positions(1, 1), real_positions(2, 1), '^', 'MarkerSize', 6, 'MarkerFaceColor',customOrange); legend_labels{end+1} = 'Starting Point';
    plot(real_positions(1,end), real_positions(2,end), '*', 'MarkerSize', 6, 'MarkerEdgeColor', customOrange); legend_labels{end+1} = 'End Point'; 
    grid on; xlabel('X [m]'); ylabel('Y [m]');
    title('2D Desired and Real Trajectories of the PRR Robot');
    hold on;
    legend(legend_labels,Location='best')


    %% plotting angles, velocities and torques as function of time

    % Define hue values excluding red (0 and 1 are red, so we use other values)
    hues = [0.1, 0.2, 0.3, 0.5, 0.6, 0.7, 0.9]; % 0.1 = yellow, 0.3 = green, 0.5 = cyan, 0.7 = blue, 0.9 = purple

    % Create a palette using full saturation and brightness
    palette = hsv2rgb([hues' ones(size(hues')) ones(size(hues'))]);
    randColor = palette(randi(length(palette)), :);
    %% Plot joint parameters as a function of time
    figure(4);
    hold on;
    legend_labels = {};%allocate place for legend
    subplot(3, 1, 1);
    plot(T, X(:, 1),'color',randColor,'LineWidth',2);
    legend_labels{end+1} = 'Real';
    hold on
    plot(D.time,D.d1,'r--','LineWidth',2)
    legend_labels{end+1} = 'Desired';
    xlabel('Time (s)'); 
    ylabel('d1 (m)');
    title('Joint 1 P');
    legend(legend_labels,Location="best");
    
    subplot(3, 1, 2);
    plot(T, X(:, 2),'color',randColor,'LineWidth',2);
    hold on
    plot(D.time,D.t2,'r--','LineWidth',2)
    xlabel('Time (s)'); 
    ylabel('theta2 (rad)'); 
    title('Joint 2 R');
    legend(legend_labels,Location="best");
    subplot(3, 1, 3);
    plot(T, X(:, 3),'color',randColor,'LineWidth',2);
    hold on
    plot(D.time,D.t3,'r--','LineWidth',2);
    xlabel('Time (s)'); 
    ylabel('theta3 (rad)'); 
    title('Joint 3 R');
    sgtitle('Joints as function of time')
    legend(legend_labels,Location='best');
    %% Plot joint linear and angular velocities as a function of time
    linear_v_desired = zeros(length(D.time),3);%allocate place for desired velocities trajectory
    angular_v_desired = zeros(length(D.time),3);%allocate place for desired velocities trajectory
    linear_v_real = zeros(length(D.time),3);%allocate place for real velocities trajectory
    angular_v_real = zeros(length(D.time),3);%allocate place for real velocities trajectory
    J_A = [    %save the linear part of the Jacobian
        0,                    0,                 0;
        0,                    0,                 0;
        0,                    1,                 1
    ];
    %find d1 t2 t3 for each move in the real & desired trajectory
    for o = 1:length(D.time) %desired
        s2t = sin(D.t2(o));c2t = cos(D.t2(o)); s23t = sin(D.t2(o) + D.t3(o));c23t = cos(D.t2(o) + D.t3(o)); %desired trajectory angles
        J_L_d=[ %desired Jacobian linear
            0 ,-3*0.01*(s2t + s23t), -3*0.01*s23t ;
            0, 3*0.01*(c2t + c23t), 3*0.01*c23t ;
            1, 0, 0];
        J_desired = [J_L_d;J_A];
        derivatives_desired_point = [0; D.v2(o); D.v3(o)]; % running on o
        v_w_desired = J_desired*derivatives_desired_point; % computation of the linear and angular velocities
        linear_v_desired(o,:) = v_w_desired(1:3)'; %save to vector
        angular_v_desired(o,:) = v_w_desired(4:6)';
    end
    for o = 1:length(T) %real
        s2t = sin(X(o,2));c2t = cos(X(o,2)); s23t = sin(X(o,2) + X(o,3));c23t = cos(X(o,2) + X(o,3)); %real trajectory angles
        J_L_r=[ %real Jacobian linear
            0 ,-3*0.01*(s2t + s23t), -3*0.01*s23t ;
            0, 3*0.01*(c2t + c23t), 3*0.01*c23t ;
            1, 0, 0];
        J_real = [J_L_r;J_A];
        derivatives_real_point = [0; X(o,5); X(o,6)]; %theta 2 dot, theta 3 dot
        v_w_real= J_real*derivatives_real_point; % get the linear and angular velocities
        linear_v_real(o,:) = v_w_real(1:3)'; %save to vector
        angular_v_real(o,:) = v_w_real(4:6)';
    end
    %Plotting linear velocities with desired and real
    figure(5);
    hold on;
    subplot(3, 1, 1); %X linear
    plot(T,linear_v_real(:,1),'color',randColor,'LineWidth',2);%linear real
    hold on
    plot(D.time,linear_v_desired(:,1),'r--','LineWidth',2);%linear desired  
    xlabel('Time (s)'); 
    ylabel('v_X (m/s)'); 
    grid on;
    title('v_X Linear');
    legend(legend_labels,Location='best'); 

    subplot(3, 1, 2);%Y linear
    plot(T,linear_v_real(:,2),'color',randColor,'LineWidth',2);%linear real
    hold on
    plot(D.time,linear_v_desired(:,2),'r--','LineWidth',2);%linear desired
    xlabel('Time (s)'); 
    ylabel('v_Y (m/s)');
    grid on;
    title('v_Y Linear');
    legend(legend_labels,Location='best');

    subplot(3, 1, 3);%Z linear
    plot(T,linear_v_real(:,3),'color',randColor,'LineWidth',2);%linear real
    hold on 
    plot(D.time,linear_v_desired(:,3),'r--','LineWidth',2);%linear desired
    xlabel('Time (s)'); 
    ylabel('v_Z (m/s)');
    grid on;
    title('v_Z Linear');
    legend(legend_labels,Location='best');
    sgtitle('Linear velocity as function of time')

    %Plotting angular velocities with desired and real
    figure(6)
    hold on;
    subplot(3, 1, 1); %X Angular
    plot(T,angular_v_real(:,1),'color',randColor,'LineWidth',2);%angular real
    hold on
    plot(D.time,angular_v_desired(:,1),'r--','LineWidth',2);%angular desired
    xlabel('Time (s)'); 
    ylabel('\omega_X (m/s)'); 
    grid on;
    title('\omega_X Angular');
    legend(legend_labels,Location='best');

    subplot(3, 1, 2); %Y Angular
    plot(T,angular_v_real(:,2),'color',randColor,'LineWidth',2);%angular real
    hold on
    plot(D.time,angular_v_desired(:,2),'r--','LineWidth',2);%angular desired
    xlabel('Time (s)'); 
    ylabel('\omega_Y (m/s)'); 
    grid on;
    title('\omega_Y Angular');
    legend(legend_labels,Location='best');

    subplot(3, 1, 3); %Z Angular
    plot(T,angular_v_real(:,3),'color',randColor,'LineWidth',2);%angular real
    hold on
    plot(D.time,angular_v_desired(:,3),'r--','LineWidth',2);%angular desired
    xlabel('Time (s)'); 
    ylabel('\omega_Z (m/s)'); 
    grid on;
    title('\omega_Z Angular');
    legend(legend_labels,Location='best');
    sgtitle('Angular velocity as function of time')

    %% Plot joint torques as a function of time
    figure(7);
    hold on
    subplot(3, 1, 1);
    plot(T, X(:, 7),'color',randColor,'LineWidth',2,'LineStyle','--');
    xlabel('Time (sec)');
    ylabel('\tau_1 (N*m)'); 
    title('Torque \tau_1');

    subplot(3, 1, 2);
    plot(T, X(:, 8),'color',randColor,'LineWidth',2,'LineStyle','--'); 
    xlabel('Time (sec)'); 
    ylabel('\tau_2 (N*m)'); 
    title('Torque \tau_2');

    subplot(3, 1, 3);
    plot(T, X(:, 9),'color',randColor,'LineWidth',2,'LineStyle','--'); 
    xlabel('Time (sec)'); 
    ylabel('\tau_3 (N*m)'); 
    title('Torque \tau_3');
    sgtitle('Torques as function of time')

    %% Plot end effector as a function of time
    figure(8);
    hold on 
    subplot(3, 1, 1);
    plot(T, real_positions(1,:)','color',randColor,'LineWidth',2);
    hold on
    plot(D.time,desired_positions(1,:)','r--','LineWidth',2);
    xlabel('Time (sec)'); 
    ylabel('X (m)'); 
    title('End effactor in X (m)');
    legend(legend_labels,Location='best');

    subplot(3, 1, 2);
    plot(T, real_positions(2,:)','color',randColor,'LineWidth',2); 
    hold on
    plot(D.time,desired_positions(2,:)','r--','LineWidth',2);
    xlabel('Time (s)'); 
    ylabel('Y (m)'); 
    title('End effactor in Y (m)');
    legend(legend_labels,Location='best');

    subplot(3, 1, 3);
    plot(T, real_positions(3,:)','color',randColor,'LineWidth',2);
    hold on
    plot(D.time,desired_positions(3,:)','r--','LineWidth',2);
    xlabel('Time (s)'); 
    ylabel('Z (m)'); 
    title('End effactor in Z (m)');
    legend(legend_labels,Location='best');
    sgtitle('End effactors as function of time')

    %% Simulation video 
    
    % Define video writer object
    videoFilename = 'robot_animation.avi';  % Name of the output video file
    videoObj = VideoWriter(videoFilename);  % Create a VideoWriter object
    videoObj.FrameRate = 3;  % Set the frame rate (adjust to control the speed)
    open(videoObj);  % Open the video file for writing
    % Calculate the total number of frames needed for a 2-second video
    total_frames = 30;  % 30 fps
    
    % Calculate the indices for downsampling your data
    indices = round(linspace(1, length(D.time), total_frames));

    % make X same size as D.time
    X_mod = interp1(linspace(T(1), T(end), size(X, 1)), X, D.time);

    %simulation
    % Plot the workspace grid
    d1=D.d1(1);
    [xc,yc]=xy_circle();
    figure;
    hold on;
    %legend_labels={};
    plot3(xc, yc, ones(1, 100) * d1, 'b--', 'LineWidth', 1); 
    plot_desired();
    legend_labels = {'Desired Trajectory', 'Real Trajectory', 'Workspace Boundary'};
    legend(legend_labels, 'Location', 'best');  % Set legend initially
    for j = 1:length(indices)
        % Extract the joint angles at the current time step from X - real
        idx = indices(j);
        d1 = X_mod(idx,1);
        t2 = X_mod(idx,2);
        t3 = X_mod(idx,3);
        DH_table_real = [0, 0, d1, 0; %real = X ( notation)
                         l1, 0, 0, t2;
                         l2, 0, 0, t3;
                         l3, 0, 0, 0];
        T_real = DH2mat(DH_table_real);
        real_positions(:, j) = T_real(1:3, 4); % Extract end-effector position

        % Clear the previous plot
        cla; 
        plot3(xc, yc, ones(1, 100) * d1, 'b--', 'LineWidth', 1); % Replot static elements
        plot_desired();  % Replot desired trajectory
        plot3(real_positions(1, 1:j), real_positions(2, 1:j), real_positions(3, 1:j), 'g-', 'LineWidth', 2);
        % Update the real trajectory plot data
        plot_workspace(d1, t2, t3);

        grid on; xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
            % Delete the existing legend
        legend_handle = findobj(gcf, 'Type', 'Legend');
        if ~isempty(legend_handle)
            delete(legend_handle);
        end
    
        % Set legend after plotting all elements
        legend({'Workspace Boundary','Desired Trajectory','Start Point','Desired End Point','Real Trajectory', 'Prismatic Joint','Link','','', 'Revolute Joint',''}, 'Location', 'northeastoutside');

       % plot_desired()
        title(sprintf('Robot Configuration at t = %.2f s', D.time(idx)));

        drawnow;
        axis equal;
        % Capture the current figure as a frame
        frame = getframe(gcf);  % Get current frame of the figure
        writeVideo(videoObj, frame);  % Write the frame to the video file
        pause(0.001); % Pause to visualize the movement (adjust time as needed)
    end

    % Close the video file
    close(videoObj);  % Close the video file after all frames have been written 
    

end 