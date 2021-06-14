function s = kalmanPredict(s)
   s.x = s.A*s.x + s.B*s.u;
   s.P = s.A * s.P * s.A' + s.Q;
   K = s.P*s.H'*inv(s.H*s.P*s.H'+s.R);
   s.P = s.P - K*s.H*s.P;
return
end