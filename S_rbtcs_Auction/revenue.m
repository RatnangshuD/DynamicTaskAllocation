function E = revenue(t,i,j,model)
if model.agent(i).allocated_task~=j
  tij=ij_time(model.Task(j).pos,model.agent(i).pos,model.agent(i).velocity);
  
else
    
  tij=0;
  
end

S=model.Task(j).s+(model.k*model.Task(j).alpha*sqrt(model.Task(j).s)-model.agent(1).task_beta(j))*tij;
E=model.k1*model.Task(j).alpha*model.agent(i).beta*sqrt(S)-0.1*model.k2*tij;

