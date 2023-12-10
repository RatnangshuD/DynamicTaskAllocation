function ag_time = ij_time(pos1,pos2,vel)
dist = norm(pos1-pos2);
ag_time = dist/vel;


