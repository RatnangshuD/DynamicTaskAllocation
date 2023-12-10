clc
clear
clf

%%
Fir_pts = [80,100; 120,100]'; % Location of the fire points
alpha = [1.2; 1.5]; % Radial velocity of fire points
s0 = [2.1; 1.2]; % Initial values of s
k = 0.6; 
ag_ip = [80,70,70,70,80,90,90,120,130,120;90,90,100,110,110,110,100,90,100,110]; % Initial locations of the agents
beta_i = [0.2,0.2,0.2,0.2,0.2,0.2,0.6,0.55,1.0,0.7]; 
vels = [60,85,60,85,60,85,60,80,10,60]; % Velocities of the agents
redep = 1;
redep2 = redep;
flag = 0;
%% Initial Deployment
Nt = length(Fir_pts); % Number of targets
Na = length(ag_ip); % Number of agents
t_mat = zeros(Na,Nt);
for i=1:Na
    for j=1:Nt
        t_mat(i,j) = norm(Fir_pts(:,j)-ag_ip(:,i))/vels(i);
    end
end
tcalc = t_mat;
%%
task = zeros(Na,Nt);

task';

%%
dt = 0.01;
t = 0:dt:2.5;
%task_Big = zeros(301,Na,Nt);
s = zeros(length(t),Nt);
bet = zeros(length(t),Nt);
bet_opt = zeros(length(t),Nt);
s(1,:) = s0;
task_Big(1,:,:) = zeros(Na,Nt); %task;

for i=2:length(t)
    for nag=1:Na
        for nfir=1:Nt
            if t(i)>tcalc(nag,nfir)
                task_Big(i,nag,nfir)=task(nag,nfir);
            end 
        end
    end
    %task_Big(i,:,:) = task;
    beta = beta_i'.*task;
    b=zeros(Nt,1);
    for nag=1:Na
        for nfir=1:Nt
            if t(i)>tcalc(nag,nfir)
                b(nfir) = b(nfir)+beta(nag,nfir);
            end
            if s(i-1,nfir) == 0
                b(nfir) = 0;
            end
        end
    end
    
    for nfir = 1:Nt
        s(i,nfir) = max(0,s(i-1,nfir) + (k*alpha(nfir)*sqrt(s(i-1,nfir))-b(nfir))*dt);
        bet(i,nfir) = b(nfir);
    end
    
    %% Redeployment
    if redep == 1 %&& t(i)> max(min(tcalc'))
        Q = sum(b);
        b_opt = calcOpt(s(i-1,:),Nt,k,alpha,Q);
        bet_opt(i,:) = b_opt;
        %b_opt = [2;2];

        OmegaR = find(b>b_opt); % Tasks that can provide Reinforcement
        OmegaN = find(b<b_opt); % Tasks that Need reinforcement
  
        AR = [];
        if ~(isempty(OmegaR))
            btemp = b(OmegaR);
            beta_temp = beta;
            beta_temp(beta_temp==0) = NaN;
            while btemp>b_opt(OmegaR)
                [bR,mR] = min(beta_temp(:,OmegaR));
                btemp = btemp-bR;
                AR = [AR;mR];
                beta_temp(mR,:) = NaN;
            end
            %AR(end)=[];
        end
        %i
        if ~(isempty(AR))
            %i
            tAR = t(i);
            newTask = OmegaN;
            redep = 0;
            flag = 1;
        end
    end
    if flag == 1
        ttravel = norm(Fir_pts(:,1)-Fir_pts(:,2))/vels(AR);
        if t(i)>tAR && t(i)<tAR + ttravel
            task(AR,:) = [0;0];
        elseif t(i)>tAR + ttravel
            task(AR,newTask) = 1;
        else
            task = task;
        end
    end
end
% figure(1)
% subplot(2,1,1)
% plot(t/dt,s)
% xlabel('t/\Delta t')
% ylabel('s_j(t)')
% legend('Fire 1', 'Fire 2')
% subplot(2,1,2)
% plot(t/dt,bet); hold on;
% % plot(t/dt,bet_opt); 
% xlabel('t/\Delta t')
% ylabel('\beta^\Sigma_j(t)')
% legend('Fire 1', 'Fire 2')

%% plot2D
figure(1)
for i=1:length(t)
    hold on
    s_fir1 = 5*s(i,1);
    s_fir2 = 5*s(i,2);
    rectangle('Position',[Fir_pts(1,1)-s_fir1/2 Fir_pts(2,1)-s_fir1/2 s_fir1 s_fir1],'Curvature',[1,1],'FaceColor','r')
    rectangle('Position',[Fir_pts(1,2)-s_fir2/2 Fir_pts(2,2)-s_fir2/2 s_fir2 s_fir2],'Curvature',[1,1],'FaceColor','r')
    task_curr = task_Big(i,:,:);
    
    sid = 10;
    
    
    if redep2==1
        if t(i)>tAR && t(i)<tAR+ttravel 
            ay = 100;
            x0 = 120-sid;
            xf = 80+sid;
            ax = x0 + (xf-x0)*(t(i)-tAR)/ttravel;
            plot(ax,ay,'ko', 'MarkerFaceColor','b', 'Markersize', 10);
        end
    end
    
    a1 = sum(task_Big(i,:,1));
    if a1>=3
        pgon1 = nsidedpoly(a1,'Center',Fir_pts(:,1)','SideLength',sid);
        %plot(pgon1,'FaceAlpha',0);
        plot(pgon1.Vertices(:,1),pgon1.Vertices(:,2), 'ko', 'MarkerFaceColor','b', 'Markersize', 10)
    elseif a1==2
        plot(Fir_pts(1,1),Fir_pts(2,1)-sid,'ko', 'MarkerFaceColor','b', 'Markersize', 10);
        plot(Fir_pts(1,1),Fir_pts(2,1)+sid,'ko', 'MarkerFaceColor','b', 'Markersize', 10);
    elseif a1==1
        plot(Fir_pts(1,1),Fir_pts(2,1)-sid,'ko', 'MarkerFaceColor','b', 'Markersize', 10);
    else
        temp=0;
    end
    
    a2 = sum(task_Big(i,:,2));
    if a2>=3
        pgon2 = nsidedpoly(a2,'Center',Fir_pts(:,2)','SideLength',sid);
        %plot(pgon2,'FaceAlpha',0);
        plot(pgon2.Vertices(:,1),pgon2.Vertices(:,2), 'ko', 'MarkerFaceColor','b', 'Markersize', 10)
    elseif a2==2
        plot(Fir_pts(1,2),Fir_pts(2,2)-sid,'ko', 'MarkerFaceColor','b', 'Markersize', 10);
        plot(Fir_pts(1,2),Fir_pts(2,2)+sid,'ko', 'MarkerFaceColor','b', 'Markersize', 10);
    elseif a2==1
        plot(Fir_pts(1,2),Fir_pts(2,2)-sid,'ko', 'MarkerFaceColor','b', 'Markersize', 10);
    else
        temp=0;
    end
        
    grid on;
    axis equal;
    xlim([60 140])
    ylim([60 140])
    text(70, 120, ['t/\Delta t = ', num2str(t(i)/dt)], 'FontSize', 14);
    pause(0.1)
    hold off;
    if i ~= length(t)
        clf;
    end
end

%%
figure(2)
h2 = subplot(2,1,1);
hold on
plot(t/dt,s(:,1),'Color','#D95319','Linewidth',2);
plot(t/dt,s(:,2),'Color','#0072BD','Linewidth',2);
xlabel('t/\Delta t')
ylabel('s_j(t)')
legend('Fire 1', 'Fire 2')
xlim([0 250])
ylim([0 2.5])
grid on

subplot(2,1,2);
hold on
plot(t/dt,bet(:,1),'Color','#D95319','Linewidth',2);
plot(t/dt,bet(:,2),'Color','#0072BD','Linewidth',2);
legend('Fire 1', 'Fire 2')
xlabel('t/\Delta t')
ylabel('\beta^\Sigma_j(t)')
xlim([0 250])
ylim([0 4])
grid on