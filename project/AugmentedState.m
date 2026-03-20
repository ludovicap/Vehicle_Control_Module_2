function [Aa,Ba,Cza,Dzua,Bwa,Dwa] = AugmentedState(A,B)

    [nx,~] = size(B);
    rho = 0.001;

    % Uscita di tracking: psi
    Cr = [0 1];
    Dr = 0;

    % Peso di performance
    Cz = [0 1]*rho; 
    Dzu = sqrt(rho);

    % Disturbo sul plant
    Bw = 0.1 * B; 
    Fr = 0;
    Fz = 0;

    % Sistema aumentato
    % xa = [edot; e; x]
    Aa = [0            0            -Cr;
          1            0            zeros(1,nx);
          zeros(nx,1)  zeros(nx,1)  A];

    Ba = [-Dr;
           0;
           B];

    % wa = [w; r]
    Bwa = [-Fr         1;
            0          0;
            Bw         zeros(nx,1)];
  
    % Output di performance
    Cza  = [0 0 Cz];
    Dzua = Dzu;
    Dwa  = [Fz 0];
end