clc
clear
%clf

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
task_curr = find(s0==max(s0));
task(:,task_curr) = ones(Na,1);
task';

%%
dt = 0.01;
t = 0:dt:5;
s = zeros(length(t),Nt);
bet = zeros(length(t),Nt);
bet_opt = zeros(length(t),Nt);
s(1,:) = s0;
task_Big(1,:,:) = zeros(Na,Nt); %task;
tAR = [];

for i=2:length(t)
    %task_Big(i,:,:) = task;
    beta = beta_i'.*task;
    b=zeros(Nt,1);
    for nag=1:Na
        for nfir=1:Nt
            if t(i)>tcalc(nag,nfir)
                task_Big(i,nag,nfir)=task(nag,nfir);
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
    
    %%
    if s(i,task_curr) < 0.001 && flag == 0
        task_prev = task_curr;
        task_curr = 3-task_prev;
        task(:,task_curr) = ones(Na,1);
        flag = 1;
        tAR = [tAR; t(i)];
    end

    if flag == 1
        for nag=1:Na
            for nfir=1:Nt
                ttravel = norm(Fir_pts(:,1)-Fir_pts(:,2))/vels(nag);
                if t(i)>=tAR(1) && t(i)<tAR(1) + ttravel
                    task(nag,:) = [0;0];
                elseif t(i)>=tAR(1) + ttravel
                    task(nag,task_curr) = 1;
                    task(nag,task_prev) = 0;
                else
                    temp = 1;
                end
            end
        end
    end

    if max(s(i,:))<0.001
        T_fin(vc) = min(t(i),T_fin(vc));
    end
end
end

plot(0.645*vel_counter, T_fin);
