function block = continuousObserverDelayLmi(A0,Ad,P,Q,R,g,delay,rate)
% continuousObserverDelayLmi Affine Schur form of the constant-matrix LKF.
% Accepts numeric matrices or symbolic SDP variables during synthesis.
    n=size(P,1);r=exp(-2*rate*delay);
    Pi=[P*A0+A0.'*P+2*rate*P+Q-r*R,P*Ad+r*R,P; ...
        Ad.'*P+r*R,-r*(Q+R),zeros(n);P,zeros(n),-g*eye(n)];
    flow=[A0,Ad,eye(n)];
    block=[Pi,delay*flow.'*R;delay*R*flow,-R];
end
