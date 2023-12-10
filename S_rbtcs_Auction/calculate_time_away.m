function f=calculate_time_away(model,j,Id,beta1)

s2=model.Task(j).s;
tij=ij_time(model.agent(Id).pos,model.Task(j).pos,model.agent(Id).velocity);
s1=s2+model.agent(1).s_dot(j)*tij;
k_alpha=model.k*model.Task(j).alpha;

f=tij+((2*beta1)/(k_alpha)^2)*log(beta1/(beta1-k_alpha*sqrt(s1)))-(2/k_alpha)*(sqrt(s1));
