ag_ps = [100,140;80,140;100,70;80,70;60,120;60,130;90,120;40,60;50,50;50,40;70,20;30,20;30,40;20,60;80,50;80,30;90,40;90,50]; % Initial locations of the agents
beta_i = [0.25,0.2,0.2,0.2,0.2,0.2,0.5,0.4,0.2,0.3,0.2,0.2,0.3,0.2,0.6,0.6,1,0.2]; 
vels = [170,120,324,300,120,135,60,120,85,60,216,135,60,170,60,80,10,85]; % Velocities of the agents

for i=1:length(ag_ps)
    
    model.agent(i).pos=ag_ps(i,:);
    model.agent(i).velocity=vels(i);
    model.agent(i).beta=beta_i(i);
    model.agent(i).Id=i;
    model.agent(i).allocated_task=0;
    model.agent(i).away_flag=1;
end

Fir_pts = [80,120;40,40;80,40]; % Location of the fire points
alpha = [1.1,1.2,1.2]; 
sinit = [1.9,1.8,1.15];

model.no_agent=length(model.agent);
for j=1:length(Fir_pts)
    
    model.Task(j).pos=Fir_pts(j,:);
    model.Task(j).alpha=alpha(j);
    model.Task(j).sinit=sinit(j);
    model.Task(j).s=sinit(j);
    model.Task(j).Id=j;
    
end

model.no_Task=length(model.Task);
t=0;
model.k=0.6;
model.k1=0.2;
model.k2=0.1;
model.dt=0.001;
model.Ts=2*model.dt;

%selecting auctioner as 1st agent
model.agent(1).no_imp_task=0;

for j=1:model.no_Task
    model.agent(1).task_beta(j)=0;
    model.agent(1).all_s(j)=model.Task(j).s;
    for i=1:model.no_agent
        if model.agent(i).allocated_task==j
            
            model.agent(1).task_beta(j)=model.agent(1).task_beta(j)+model.agent(i).beta;
            
            
        end
        
    end
    model.agent(1).s_dot(j)=calculate_execution_cap(model,j);
    if model.agent(1).s_dot(j)>=0
        
        model.agent(1).no_imp_task=model.agent(1).no_imp_task+1;
        model.agent(1).imp_task_id(model.agent(1).no_imp_task)=j;
        
    end
    
    
    
end

while model.agent(1).no_imp_task>0
    model.agent(1).un_allocated=0;
    model.agent(1).un_allocated_Id=[];
    for i=1:model.no_agent


            model.agent(1).all_velocity(i)=model.agent(i).velocity;
            model.agent(1).all_pos(i,:)=model.agent(i).pos;
            
            if model.agent(i).allocated_task==0
                model.agent(1).un_allocated=model.agent(1).un_allocated+1;
                model.agent(1).un_allocated_Id(model.agent(1).un_allocated)=i;


            end        



    end




    
    model.agent(1).bid_matrix=zeros(model.agent(1).no_imp_task,model.agent(1).un_allocated);
    model.agent(1).reach_time=zeros(model.agent(1).no_imp_task,model.agent(1).un_allocated);

    for i=1:model.agent(1).un_allocated
        Id=model.agent(1).un_allocated_Id(i);
        for j=1:model.agent(1).no_imp_task
            task_Id=model.agent(1).imp_task_id(j);
            model.agent(1).bid_matrix(j,i)=revenue(t,Id,task_Id,model);
            model.agent(1).reach_time(j,i)=ij_time(model.agent(Id).pos,model.Task(task_Id).pos,model.agent(Id).velocity);
            
            
            
        end
        
        
    end
    
    
    
    
    [maxValues, columnIds] = max(model.agent(1).bid_matrix, [], 2);
    
    while ~checkUniqueness(columnIds)
    
        dummy_column=columnIds;

        i=1;

        while i<=length(dummy_column)
            dummy_column_ids=[];
            dummy_column_ids=find(dummy_column==dummy_column(i));

            if length(dummy_column_ids)>1
                col=[];

                for j=1:length(dummy_column_ids)

                   col(j)=model.agent(1).reach_time(dummy_column_ids(j),dummy_column(j));

                end
                [~,min_id]=min(col);
                
              
                
                for k=1:length(dummy_column_ids)
                    if k~=min_id
                   
                        model.agent(1).bid_matrix(dummy_column_ids(k),dummy_column(k))=-100*rand(1);
                        dummy_column(dummy_column_ids(k))=-100*rand(1);
                        
                        
                    end
                                   
                    
                    


                end
                

              
            end

            i=i+1;

        end
        
        [maxValues, columnIds] = max(model.agent(1).bid_matrix, [], 2);

    end

    %% 

    
    j=1;
   
    while j<=length(columnIds)
        
       
        
       Id=model.agent(1).un_allocated_Id(columnIds(j));
       
       task_id=model.agent(1).imp_task_id(j);

       
       model.agent(Id).allocated_task=task_id;
       

       
       
  
       
       model.agent(1).task_beta(task_id)=model.agent(1).task_beta(task_id)+model.agent(Id).beta;
       
       model.agent(1).s_dot(task_id)=calculate_execution_cap(model,task_id);
       


       
       
     j=j+1;   
    end
    j=1;
    
    while j<=length(columnIds)
        
         task_id=model.agent(1).imp_task_id(j);
        
         if model.agent(1).s_dot(task_id)<0

            model.agent(1).imp_task_id(j)=[];

           



            columnIds(j)=[];
            
            columnIds(j:length(columnIds))=columnIds(j:length(columnIds))-1;
     



            model.agent(1).no_imp_task=model.agent(1).no_imp_task-1;
            
            j=j-1;

         end
         
        j=j+1; 

    end
    
    
end

%% 

model.agent(1).un_allocated=0;
model.agent(1).un_allocated_Id=[];
for i=1:model.no_agent



        if model.agent(i).allocated_task==0
            model.agent(1).un_allocated=model.agent(1).un_allocated+1;
            model.agent(1).un_allocated_Id(model.agent(1).un_allocated)=i;


        end        



end
model.agent(1).bid_matrix=zeros(model.no_Task,model.agent(1).un_allocated);

for i=1:model.agent(1).un_allocated
    
    Id=model.agent(1).un_allocated_Id(i);
    
    for j=1:model.no_Task

        model.agent(1).bid_matrix(j,i)=revenue(t,Id,j,model);
        


    end
    
    
end



[~, rowIds] = max(model.agent(1).bid_matrix, [], 1);   

i=1;

while i<=model.agent(1).un_allocated
    
    Id=model.agent(1).un_allocated_Id(i);
    
    model.agent(Id).allocated_task=model.Task(rowIds(i)).Id;
    
    model.agent(1).task_beta(rowIds(i))=model.agent(1).task_beta(rowIds(i))+model.agent(Id).beta;
    
    
    
    i=i+1;
end

model.agent(1).un_allocated_Id(:)=[];
model.agent(1).un_allocated=0;

model.agent(1).allocation_matrix=zeros(model.no_Task,model.no_agent);

tracking_vector=model.agent(1).all_pos*0;
for j=1:model.no_Task
    
    model.agent(1).s_dot(j)=calculate_execution_cap(model,j);
    
    for i=1:model.no_agent
        if model.agent(i).allocated_task==j
            
            model.agent(1).allocation_matrix(j,i)=1;
            model.agent(i).task_pos=model.Task(j).pos;
            tracking_vector(i,:)=model.Task(j).pos-model.agent(i).pos;
            tracking_vector(i,:)=tracking_vector(i,:)/norm(tracking_vector(i,:));
            
        
        end
        
        
    end
    
    
end
%% Integrating till reaches the fire

model.agent(1).actual_beta=0*model.agent(1).task_beta;

model.agent(1).flame_radius=(model.agent(1).all_s/pi).^0.5;

Ts_counter=0;
S_flag=0;
count=1;

F1=zeros(1,model.no_agent);
 model.agent(1).pos_t=zeros(100/model.dt,model.no_agent,2);
for t=0:model.dt:100
    if Ts_counter==0 || S_flag ==1
        
        for i=1:model.no_agent
               task_Id=model.agent(i).allocated_task;
               model.agent(i).pos=model.agent(1).all_pos(i,:);
            if (norm(model.agent(i).task_pos-model.agent(i).pos)-model.agent(1).flame_radius(task_Id))<0
               if model.agent(i).away_flag==1  
                   model.agent(i).away_flag=0;
                   model.agent(1).actual_beta(task_Id)=model.agent(1).actual_beta(task_Id)+model.agent(i).beta;
                   F1(i)=1;
               end
                
            end
            
            
        end
        
        
        for j=1:model.no_Task
            
           model.agent(1).s_dot(j)=calculate_actual_execution_cap(model,j);
           model.agent(1).flame_radius(j)=(model.agent(1).all_s(j)/pi)^0.5;
        end
        
        for i=1:model.no_agent
            
           S_dot(i)=model.agent(1).s_dot(model.agent(i).allocated_task) ;
           flame_radii(i)=model.agent(1).flame_radius(model.agent(i).allocated_task);
            
        end
        
        
        velocity_mat=model.agent(1).all_velocity.*(1-F1)-S_dot./(2*pi*flame_radii).*(F1);
        
        Ts_counter=Ts_counter+1;
        
        S_flag=0;
        
    elseif Ts_counter==model.Ts/model.dt-1
        
        Ts_counter=0;
        
        
    else
        
        
         Ts_counter=Ts_counter+1;   
    end 
    
    
        model.agent(1).track_vec(count,:,:)=tracking_vector;
        model.agent(1).integrated_beta(count,:)=model.agent(1).actual_beta;
        
        model.agent(1).s_t(count,:)=model.agent(1).all_s + model.agent(1).s_dot*model.dt;
        model.agent(1).all_s=model.agent(1).s_t(count,:);
        
        model.agent(1).pos_t(count,:,:)=model.agent(1).all_pos+velocity_mat'.*tracking_vector*model.dt;
        
        model.agent(1).all_pos=squeeze(model.agent(1).pos_t(count,:,:));
        
        
        
       termination_flag=sum(model.agent(1).all_s<0) ;
       
    if termination_flag~=model.no_Task
        if termination_flag>0 

            remaining_tasks=find(model.agent(1).all_s>0);
            completed_tasks=find(model.agent(1).all_s<=0);

            for j=1:length(completed_tasks)

                for i=1:model.no_agent

                    if model.agent(i).allocated_task==completed_tasks(j)

                        model.agent(1).un_allocated=model.agent(1).un_allocated+1;
                        model.agent(1).un_allocated_Id(model.agent(1).un_allocated)=i;


                    end


                end

                 model.agent(1).s_dot(completed_tasks(j))=0;
                 model.agent(1).actual_beta(completed_tasks(j))=0;
                 model.Task(completed_tasks(j)).s=0;




            end
            model.agent(1).bid_matrix=zeros(length(remaining_tasks),model.agent(1).un_allocated);

            for i=1:model.agent(1).un_allocated
             Id=model.agent(1).un_allocated_Id(i);
             model.agent(Id).pos=model.agent(1).all_pos(Id,:);
                 for j=1:length(remaining_tasks)

                       model.agent(1).bid_matrix(j,i)=revenue(t,Id,remaining_tasks(j),model);




                 end


            end


            [~, rowIds] = max(model.agent(1).bid_matrix, [], 1);   

            i=1;


            while i<=model.agent(1).un_allocated

                Id=model.agent(1).un_allocated_Id(i);

                model.agent(Id).allocated_task=remaining_tasks(rowIds(i));
                model.agent(Id).away_flag=1;
                model.agent(Id).task_pos=model.Task(remaining_tasks(rowIds(i))).pos;
                tracking_vector(Id,:)=model.Task(remaining_tasks(rowIds(i))).pos-model.agent(Id).pos;
                tracking_vector(Id,:)=tracking_vector(Id,:)/norm(tracking_vector(Id,:));
                F1(Id)=0;


                i=i+1;
            end

            model.agent(1).un_allocated_Id(:)=[];
            model.agent(1).un_allocated=0;



            S_flag=1;

        end
    else    
        break
    end
        
        
        count=count+1;  
    
    
end    




    




%% Dynamic

% t=t+model.Ts;

Q=sum(beta_i);

    
    


x0 = [0,model.agent(1).task_beta];
% 
% xo(model.notask
% 
% % Create options structure with the desired tolerance
options = optimset('TolFun', 1e-2, 'TolX', 1e-1);

args={model,Q};
% 
% % Solve the equations with the specified tolerance
solution = fsolve(@(x) optimizer(x, args{:}), x0, options);


T=solution(1);
beta_star=solution(2:model.no_Task+1);



available_task=find(model.agent(1).task_beta > beta_star);

unavailable_task=find(model.agent(1).task_beta <= beta_star);

surplus_deficit=model.agent(1).task_beta - beta_star;

for j=1:length(available_task)
    surplus=surplus_deficit(available_task(j));
    for i=1:model.no_agent
       if model.agent(i).beta < surplus && model.agent(i).allocated_task==available_task(j)
          model.agent(1).un_allocated= model.agent(1).un_allocated+1;
          model.agent(1).un_allocated_Id(model.agent(1).un_allocated)=i;

      
       end
    end
    
    
end

allocation_loop=0;



while allocation_loop==0
    for j=1:model.no_Task
        
      

       model.agent(1).time_taken(j)=calculate_time(model,j,model.agent(1).task_beta(j)); 

    end
    [~,model.agent(1).max_time_task_Id]=max(model.agent(1).time_taken);

    count=1;
    model.agent(1).no_bidder=0;

    for i=1:model.agent(1).un_allocated
        Id=model.agent(1).un_allocated_Id(i);
         E1=revenue(t,Id,model.agent(1).max_time_task_Id,model);
         E2=revenue(t,Id,model.agent(Id).allocated_task,model);

         if E1>E2

             model.agent(1).bidder_Id(count)=Id;
             model.agent(1).no_bidder=count;


             count=count+1;

         end



    end
    
    

    if model.agent(1).no_bidder>0
       minimum_time=0;
       j=model.agent(1).max_time_task_Id;
       Tji=zeros(1,model.agent(1).no_bidder);
       Tjn=Tji;

       for i=1:model.agent(1).no_bidder
          Id=model.agent(1).bidder_Id(i);
          bidder_task_Id=model.agent(Id).allocated_task;
          Tji(i)=calculate_time(model,j,model.agent(1).task_beta(bidder_task_Id)-model.agent(Id).beta);    
          Tjn(i)=calculate_time_away(model,j,Id,model.agent(1).task_beta(j)+model.agent(Id).beta);   

       end

        [new_total_time,row]=min(Tji+Tjn);

        best_bidder_Id=model.agent(1).bidder_Id(row);

        bidder_task_Id=model.agent(best_bidder_Id).allocated_task;

        Present_total_time=model.agent(1).time_taken(j)+model.agent(1).time_taken(bidder_task_Id);


        if new_total_time < Present_total_time
            model.agent(best_bidder_Id).allocated_task=j;
            model.agent(1).task_beta(bidder_task_Id)=model.agent(1).task_beta(j)-model.agent(Id).beta;
            model.agent(1).task_beta(j)=model.agent(1).task_beta(j)+model.agent(Id).beta;
            model.agent(1).bidder_Id=[];
            model.agent(1).no_bidder=0;
            model.agent(1).un_allocated_Id(model.agent(1).un_allocated_Id==best_bidder_Id)=[];
            model.agent(1).un_allocated=model.agent(1).un_allocated-1;
            Present_total_time=new_total_time;

        end
    else
        
       allocation_loop=1;
       
    end

end






