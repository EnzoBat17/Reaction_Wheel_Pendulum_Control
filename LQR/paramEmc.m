clc;
clear all;

%% 1. Parâmetros Físicos do Pêndulo
m_haste = 0.038;
m_roda = 0.179;
m_motor = 0.213;
m = (m_haste + m_roda + m_motor)*1.2;                % Massa total (kg) (20% de tolerância)

Bh = 0.0015;                                         % Atrito viscoso no pivô da base (haste) (+0,001 de tolerancia)                      (estimado) 
Br = 0.0012;                                         % Atrito viscoso na roda (+0,001 de tolerancia)                                      (estimado)
Bm = 0.0012;                                         % Atrito viscoso no eixo do motor (+0,001 de tolerancia)                             (estimado)

lr = 0.10;                                          % Distância entre o eixo da haste (pivô da base) e o centro da roda de inércia [m]
lh = 0.15144;                                        % Comprimento total da haste
lb = lh/2;                                           % Distância do centro de massa da haste até o eixo da haste (pivô da base)           (considera-se barra uniforme) [m]

g = 9.81;                                            % Aceleração da gravidade (m/s^2)

R_roda_ext = 0.07;                                   % Raio da roda de reação (m)
R_roda_int = 0.055;                                  
R_motor = 0.0178;                                    % Raio do motor (m)

Jh = (m_haste * lh^2)/3;                            %Momento de inercia da haste com referencial no pivô da base                           (haste uniforme)
Jr = 0.5*m_roda*(R_roda_int^2 + R_roda_ext^2);                           %Momento de inercia da rotação da roda em seu proprio eixo                             (anel uniforme)
Jr_trans = Jr + (m_roda*lr^2);                      %Momento de inercia da roda com referencial no pivô da base                
Jm_trans = (0.5 * (m_motor*R_motor^2)) + m_motor*lr^2;  %Momento de inercia do motor com referencial no pivô da base                           (cilindro)
Jp = Jh + Jr_trans + Jm_trans;                      %Momento de inercia do sistema com referencial no pivô da base

k_mgl = ((m_haste*lb)+(m_motor+m_roda)*lr) * g;

%% 1.1 Parâmetros Elétricos do Motor
R = 0.59; % Resistência da armadura (Ω)
Vnom = 12; %Tensão nominal do motor [V]
RPM_no_load = 18200;  %velocidade do motor sem carga [RPM]
wo = RPM_no_load * (2*pi/60); %velocidade do motor sem carga [rad/s]
ke = Vnom/wo;
kt = ke;

%% 2. Matrizes do Espaço de Estados (Planta)
% Dinâmica do sistema
A = [0,           1,          0;
    (k_mgl)/Jp, -Bh/Jp,      ((kt * ke/R) + Bm + Br)/Jp;
    -(k_mgl)/Jp, Bh/Jp,     -((Jp + Jr)/(Jp* Jr)) * ((kt * ke/R) + Bm + Br)];

% Matriz de entrada (Torque)
B = [0;
    -kt/(R*Jp);
    ((Jp+Jr)/(Jp*Jr))*(kt/R)];

% Matriz de saída
% A matriz C será uma matriz identidade 3x3.
C = eye(3);

% Matriz de transmissão direta (Nula)
D = [0];

%% 3. Condição Inicial
% Vamos simular soltando o robô com uma inclinação de ~12 graus
x0 = [deg2rad(12); 0; 0];

%% 4. Ganhos do Controlador

%Polos em malha aberta (matriz A)
polos_ma = eig(A);
disp('Polos em Malha Aberta:');
disp(polos_ma);

%numeros mais prox de 0 indicam menor peso
polos_desejados = [-1, -4, -5];

%% 5. Discretização utilizando ZOH (Zero-Order Hold)     %O sistema discreto tem a forma x[k+1] = x[k]*e^(A*Ts) + u[k]*∫(e^(A*t)*B) dt
Ts = 0.001;                                              % Freq de amostragem (1000 Hz)                                   (2*Freq de Nyquist)

%Cálculo de Ad e Bd
Ad = expm(A*Ts);                                         % e^(A*Ts)
Bd = integral(@(t) expm(A*t)*B,0,Ts,'ArrayValued',true); % ∫(e^(A*t)*B) dt

%Matrizes de saída
Cd = C;
Dd = D;

%% 6. Ganho do sistema discreto - Pole Placement
polos_desejados_discretos = exp(polos_desejados*Ts);      % e^(p*Ts)

%O MATLAB calcula os ganhos para os polos discretizados
Kd = place(Ad, Bd, polos_desejados_discretos);


%% 7. Verificando se os polos estão certos (PODE TIRAR ESSA PARTE DEPOIS SE QUISER)
polos_mf_d = eig(Ad - Bd*Kd);

disp('Polos em malha fechada discreta (z):');
disp(polos_mf_d);

disp('Ganho discreto')
disp(Kd)

%% 8. Projeto do Controlador LQR Discreto
% Matriz dos estados (Q)
q_theta  = 42;   % Penaliza o erro de ângulo da haste (theta)
q_dtheta = 0.68;    % Penaliza a velocidade de queda da haste (amortecimento)
q_wR     = 0.001;   % Penaliza a velocidade da roda (evita que o motor dispare)

Q = diag([q_theta, q_dtheta, q_wR]);

%Matriz das entradas (R)
% Aumentar R_lqr deixa o motor mais suave e reduz picos de aceleração/corrente
R_lqr = [1];

[Kd_LQR, S, polos_LQR] = dlqr(Ad, Bd, Q, R_lqr);

disp('Polos discretos em malha fechada LQR (z):');
disp(polos_LQR);

disp('Ganho discreto - LQR')
disp(Kd_LQR)


disp('Matrizes e parâmetros carregados com sucesso! Pode abrir o Simulink.');