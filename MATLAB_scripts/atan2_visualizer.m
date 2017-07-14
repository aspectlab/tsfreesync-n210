%% atan2_visualizer

q = (-1:1e-3:1);
i = 1;

tan = atan2(q,i);

figure
plot(q,q)
hold on
plot(q,tan)
plot(q,tan-q)