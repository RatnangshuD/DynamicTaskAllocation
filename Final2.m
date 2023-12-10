clc
clear

%%
Fir_pts = [80,100; 120,100]'; % Location of the fire points
alpha = [1.5; 1.8]; % Radial velocity of fire points
s0 = [2.1; 1.8]; % Initial values of s
k = 0.6; 
ag_ip = [90, 90, 90, 90, 90, 90, 90, 90, 90, 90; 100, 100, 100, 100, 100, 100, 100, 100, 100, 100];% Initial locations of the agents
beta_i = [0.45, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4]; 
vels = [100, 100, 80, 80, 80, 80, 80, 80, 80, 80]; % Velocities of the agents
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
while (min(t_mat,[],'all')<100)
    [m,n] = find(t_mat==min(min(t_mat)));
    [m,v] = min(m);
    n = n(v);
    task(m,n) = 1;
    t_mat(m,:) = Inf;
    if sum(beta_i'.*task(:,n)) > k*alpha(n)*sqrt(s0(n))
        t_mat(:,n) = Inf;
    end
end 
ua = find(sum(task,2)==0); % unallocated agents
beta0 = sum(beta_i'.*task,1); % assigned beta so far
while ~(isempty(ua))
    t_com = zeros(Nt,1);
    for i=1:Nt
        % completion time
        t_com(i) = 2*beta0(i)/(k*alpha(i))^2*log(beta0(i)/(beta0(i)-k*alpha(i)*sqrt(s0(i))))-2/(k*alpha(i))*sqrt(s0(i));
    end
    n = find(t_com==max(t_com)); % Last completed task takes precedence
    m = ua((tcalc(ua,n)==min(tcalc(ua,n)))); % Nearest agent assigned to last completed task
    task(m,n) = 1;
    ua = find(sum(task,2)==0); % unallocated agents
    beta0 = sum(beta_i'.*task,1); % assigned beta so far
end
task(10,:) = [1, 0];
task'

%%
dt = 0.01;
t = 0:dt:4;
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
        b_opt = calcOpt2(s(i-1,:),Nt,k,alpha,Q);
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
% %plot(t/dt,bet_opt); 
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
        if t(i)>=tAR && t(i)<tAR+ttravel
            ay = 100;
            x0 = 80+sid;
            xf = 120-sid;
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
    pause(0.01)
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
xlim([0 400])
ylim([0 2.5])
grid on

subplot(2,1,2);
hold on
plot(t/dt,bet(:,1),'Color','#D95319','Linewidth',2);
plot(t/dt,bet(:,2),'Color','#0072BD','Linewidth',2);
legend('Fire 1', 'Fire 2')
xlabel('t/\Delta t')
ylabel('\beta^\Sigma_j(t)')
xlim([0 400])
ylim([0 4])
grid on
plot(t/dt, k*alpha(1)*sqrt(s(:,1)),'LineStyle','--','Color','#D95319','Linewidth',2)
plot(t/dt, k*alpha(2)*sqrt(s(:,2)),'LineStyle','--','Color','#0072BD','Linewidth',2)