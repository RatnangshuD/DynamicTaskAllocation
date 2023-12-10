function b_opt = calcOpt(s,Nt,k,alpha,Q)

% syms b1 b2 t real positive
% eqn1 = t == 2*b1/(k*alpha(1))^2*log(b1/(b1-k*alpha(1)*sqrt(s(1))))-2/(k*alpha(1))*sqrt(s(1));
% eqn2 = t == 2*b2/(k*alpha(2))^2*log(b2/(b2-k*alpha(2)*sqrt(s(2))))-2/(k*alpha(2))*sqrt(s(2));
% eqn3 = b1 + b2 == Q;
% eqns = [eqn1 eqn2 eqn3];
% S = vpasolve(eqns,[b1,b2,t]);
% b_opt = [S.b1; S.b2];
b2 = max(2,s(2)+1.1478*sqrt(s(2)));
b_opt = [2;b2];
end