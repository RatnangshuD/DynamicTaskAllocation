function f=calculate_actual_execution_cap(model,j)

f=model.k*model.Task(j).alpha*sqrt(abs(model.Task(j).s))-model.agent(1).actual_beta(j);