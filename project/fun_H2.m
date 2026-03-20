function K_H2 = fun_H2(A, B1, B2, C2, D22)
% fun_H2
% -------------------------------------------------------------------------
% Static H2 state-feedback synthesis (continuous-time) using Schur LMIs.
%
% Plant:
%   x_dot = A x + B1 u + B2 w
%   z     = C2 x + D22 u
%
% Static controller:
%   u = K x
%
% Change of variables:
%   X = P^{-1},   X = X' > 0
%   Y = K X
%   K = Y X^{-1}
% -------------------------------------------------------------------------

    % Dimensions
    n  = size(A,1);
    m  = size(B1,2);
    nw = size(B2,2);
    nz = size(C2,1);

    % Decision variables
    X = sdpvar(n);
    Y = sdpvar(m,n);
    Q = sdpvar(nz);

    % Constraints exactly as in the working version
    C1 = X >= 1e-5;
    C2_lmi = Q >= 1e-5;

    C3 = ([(A*X + B1*Y) + (A*X + B1*Y)'    B2; ...
                            B2'            -eye(nw)] <= -1e-5);

    C4 = ([Q                  (C2*X + D22*Y); ...
           (C2*X + D22*Y)'          X      ] >= 1e-5);

    con = C1 + C2_lmi + C3 + C4;

    % Optimization
    opts = sdpsettings('solver','sedumi','verbose',0);
    optimize(con, trace(Q), opts);

    % Solution
    Xsol = double(X);
    Ysol = double(Y);

    K_H2 = Ysol * inv(Xsol);
end