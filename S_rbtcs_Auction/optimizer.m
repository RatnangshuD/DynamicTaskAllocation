function f=optimizer(x,model,Q)

for j=1:model.no_Task
    
    fun(j)=calculate_T_exp(model,j,x(j+1),x(1));
    
end

fun(j+1)=sum(x(2:model.no_Task+1))-Q;

f=fun;