function f=calculate_time(model,j,beta1)

s1=model.Task(j).s;
k_alpha=model.k*model.Task(j).alpha;

f=((2*beta1)/(k_alpha)^2)*log(beta1/(beta1-k_alpha*sqrt(s1)))-(2/k_alpha)*(sqrt(s1));