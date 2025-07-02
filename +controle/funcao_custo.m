function f = funcao_custo(x0, yref, Ts, t, Gz)

    Kp = x0(1);
    Ki = x0(2);
    Kd = x0(3);
    
    Gc = pid(Kp,Ki,Kd,0,Ts,'IFormula','BackwardEuler');
    tempo = 0:Ts:t-Ts;
    y = step(feedback(Gc*Gz,1),tempo);

    f = sqrt(sum(abs(y-yref).^2));
end