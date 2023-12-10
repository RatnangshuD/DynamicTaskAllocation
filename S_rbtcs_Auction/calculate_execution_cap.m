function f=calculate_execution_cap(model,j)

f=model.k*model.Task(j).alpha*sqrt(model.Task(j).s)-model.agent(1).task_beta(j);

