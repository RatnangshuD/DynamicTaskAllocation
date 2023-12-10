clc
clear
clf

%%
vel_counter = 1:50;
T_fin = 100*ones(length(vel_counter),1);
for vc = 1:length(vel_counter)
Fir_pts = [80,100; 120,100]'; % Location of the fire points
alpha = [1.2; 1.5]; % Radial velocity of fire points
s0 = [2.1; 1.2]; % Initial values of s
k = 0.6; 
ag_ip = [80,70,70,70,80,90,90,120,130,120;90,90,100,110,110,110,100,90,100,110]; % Initial locations of the agents
beta_i = [0.2,0.2,0.2,0.2,0.2,0.2,0.6,0.55,1.0,0.7]; 
vels = vel_counter(vc)*[60,85,60,85,60,85,60,80,10,60]; % Velocities of the agents
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
        AR = AR(1);
        ttravel = norm(Fir_pts(:,1)-Fir_pts(:,2))/vels(AR);
        if t(i)>tAR && t(i)<tAR + ttravel
            task(AR,:) = [0;0];
        elseif t(i)>tAR + ttravel
            task(AR,newTask) = 1;
        else
            task = task;
        end
    end

    if max(s(i,:))<0.001
        T_fin(vc) = min(t(i),T_fin(vc));
    end
end
end

plot(0.645*vel_counter, T_fin);
hold on;