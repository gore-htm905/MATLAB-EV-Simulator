function EV_With_Accuracy_Validation

clc;

%% ================= UI =================
fig = uifigure('Name','EV Simulator with Accuracy Validation',...
    'Position',[100 100 1300 750]);

%% Inputs
uilabel(fig,'Position',[20 700 200 22],'Text','Drive Cycle');
cycleDrop = uidropdown(fig,'Position',[20 670 200 30],...
    'Items',{'Constant','City','Highway'});

uilabel(fig,'Position',[20 630 200 22],'Text','Vehicle Mass (kg)');
massField = uieditfield(fig,'numeric','Position',[20 600 200 30],'Value',1500);

uilabel(fig,'Position',[20 560 200 22],'Text','Battery Capacity (kWh)');
batteryField = uieditfield(fig,'numeric','Position',[20 530 200 30],'Value',30);

uilabel(fig,'Position',[20 490 200 22],'Text','Road Slope (deg)');
slopeField = uieditfield(fig,'numeric','Position',[20 460 200 30],'Value',0);

uilabel(fig,'Position',[20 420 200 22],'Text','Initial SOC (%)');
socField = uieditfield(fig,'numeric','Position',[20 390 200 30],'Value',90);

runBtn = uibutton(fig,'push',...
    'Text','Run Simulation',...
    'Position',[20 330 200 45],...
    'ButtonPushedFcn',@runSimulation);

%% Accuracy Output Labels
rmseLabel = uilabel(fig,'Position',[20 280 350 25],'Text','Speed RMSE: ');
energyBalLabel = uilabel(fig,'Position',[20 250 350 25],'Text','Energy Balance Error: ');
effLabel = uilabel(fig,'Position',[20 220 350 25],'Text','Average Efficiency: ');
rangeLabel = uilabel(fig,'Position',[20 190 350 25],'Text','Estimated Range: ');
consLabel = uilabel(fig,'Position',[20 160 350 25],'Text','Energy Consumption: ');

%% Axes (4)
ax1 = uiaxes(fig,'Position',[350 420 400 250]);
title(ax1,'Speed Tracking');

ax2 = uiaxes(fig,'Position',[800 420 400 250]);
title(ax2,'Battery SOC');

ax3 = uiaxes(fig,'Position',[350 80 400 250]);
title(ax3,'Motor Power (kW)');

ax4 = uiaxes(fig,'Position',[800 80 400 250]);
title(ax4,'Cumulative Energy (kWh)');

%% ================= SIMULATION =================
    function runSimulation(~,~)

        %% Inputs
        m = massField.Value;
        Capacity_kWh = batteryField.Value;
        slope_deg = slopeField.Value;
        SOC = socField.Value/100;
        initial_SOC = SOC;
        cycleType = cycleDrop.Value;

        %% Constants
        Cd = 0.29;
        A = 2.2;
        rho = 1.2;
        Cr = 0.015;
        g = 9.81;
        V_nom = 350;
        R_internal = 0.05;
        eta = 0.92;
        regen_eff = 0.65;

        theta = slope_deg*pi/180;

        Q = (Capacity_kWh*1000*3600)/V_nom;

        dt = 0.1;
        t = 0:dt:600;
        n = length(t);

        v = zeros(1,n);
        SOC_vec = zeros(1,n);
        P_vec = zeros(1,n);

        energy_elec = 0;
        energy_mech = 0;
        distance = 0;

        %% Drive Cycle
        switch cycleType
            case 'Constant'
                v_ref = 20*ones(1,n);
            case 'City'
                v_ref = 15 + 8*sin(0.05*t);
            case 'Highway'
                v_ref = 25 + 5*sin(0.02*t);
        end

        %% Simulation Loop
        for k = 2:n

            error = v_ref(k-1) - v(k-1);
            Kp = 700;
            F_trac = Kp * error;
            F_trac = max(min(F_trac,6000),-6000);

            F_drag = 0.5*rho*A*Cd*v(k-1)^2;
            F_roll = m*g*Cr*cos(theta);
            F_slope = m*g*sin(theta);

            F_net = F_trac - F_drag - F_roll - F_slope;
            a = F_net/m;

            v(k) = v(k-1) + a*dt;
            if v(k) < 0
                v(k) = 0;
            end

            P_mech = F_trac * v(k);
            energy_mech = energy_mech + P_mech*dt;

            if P_mech < 0
                P_elec = P_mech * regen_eff;
            else
                P_elec = P_mech / eta;
            end

            I = P_elec / V_nom;
            V_actual = V_nom - I*R_internal;
            P_actual = V_actual * I;

            P_vec(k) = P_actual/1000;

            SOC = SOC - (I*dt)/Q;
            SOC_vec(k) = SOC*100;

            energy_elec = energy_elec + P_actual*dt;
            distance = distance + v(k)*dt;
        end

        %% ================= ACCURACY METRICS =================

        % 1 Speed RMSE
        RMSE = sqrt(mean((v - v_ref).^2));

        % 2 Energy from SOC drop
        SOC_drop = initial_SOC - SOC;
        energy_from_SOC = SOC_drop * Capacity_kWh; % kWh

        energy_calc_kWh = energy_elec/(1000*3600);

        energy_balance_error = abs(energy_from_SOC - energy_calc_kWh);

        % 3 Efficiency estimation
        if energy_elec ~= 0
            avg_eff = energy_mech / energy_elec;
        else
            avg_eff = 0;
        end

        % 4 Consumption & Range
        distance_km = distance/1000;

        if distance_km > 0
           Wh_per_km = (energy_calc_kWh*1000)/distance_km;

    usable_capacity_kWh = Capacity_kWh * initial_SOC; 

    range_est = (usable_capacity_kWh*1000)/Wh_per_km;
        else
            Wh_per_km = 0;
            range_est = 0;
        end

        %% ================= PLOTS =================
        plot(ax1,t,v,'b','LineWidth',1.5); hold(ax1,'on');
        plot(ax1,t,v_ref,'r--','LineWidth',1.5);
        hold(ax1,'off'); legend(ax1,'Vehicle','Reference');

        plot(ax2,t,SOC_vec,'LineWidth',1.5);
        ylabel(ax2,'SOC (%)');

        plot(ax3,t,P_vec,'LineWidth',1.5);
        ylabel(ax3,'Power (kW)');

        plot(ax4,t,cumsum(P_vec)*dt/3600,'LineWidth',1.5);
        ylabel(ax4,'Energy (kWh)');

        %% ================= DISPLAY RESULTS =================
        rmseLabel.Text = sprintf('Speed RMSE: %.3f m/s',RMSE);
        energyBalLabel.Text = sprintf('Energy Balance Error: %.3f kWh',energy_balance_error);
        effLabel.Text = sprintf('Average Efficiency: %.2f',avg_eff);
        rangeLabel.Text = sprintf('Estimated Range: %.1f km',range_est);
        consLabel.Text = sprintf('Energy Consumption: %.1f Wh/km',Wh_per_km);

    end

end