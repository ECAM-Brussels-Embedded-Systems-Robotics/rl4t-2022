clear all
% Sampling period

Ts = 0.01;

% General parameters

g = 9.81;       % m/s²

% Pendulum parameters
% EduMiP

L = 0.048;                  % m
D = 0.074;                  % m
R = 0.034;                  % m
m_f = 0.180;                % kg
m_w = 0.027;                % kg
I_fz = 0.000263;            % kg m²
I_fy = 0.0003;              % kg m²
I_m = 3.6e-8;               % kg m²
c_t = 0.0001;               % kg m² / s
V_b = 7.4;                  % V
n = 35.5;                   % Gearbox ratio
tau_s = 0.003;              % Nm
omega_f = 1760;             % rad / s

% Calculated parameters

I_w = m_w*R^2/2 + n^2*I_m;  % kg m²
k = tau_s/omega_f;
I_phi = 2*I_w + (2*m_w + m_f)*R^2;
I_theta = I_fz + L^2*m_f;
a = n*tau_s;
b = R*L*m_f;
c = 2*(n^2*k + c_t);
e = L*m_f*g;
c_gamma = n*tau_s*D/(2*R*I_fy);



% Linear unstable state-space (theta(0) = 0)
% Continous
ALU = [
         0    1.0000         0         0;
  185.1289  -17.1719         0   17.1719;
         0         0         0    1.0000;
 -138.5728   24.3104         0  -24.3104];
BLU = [
         0         0;
 -406.7353 -406.7353;
         0         0;
  575.8193  575.8193];
CL = eye(4);
DL = zeros(4,2);

% Discrete
lin_unstable = ss(ALU, BLU, CL, DL);
lin_unstable_d = c2d(lin_unstable, Ts, 'zoh');
ALU_d = lin_unstable_d.a;
BLU_d = lin_unstable_d.b;
CL_d = lin_unstable_d.c;
DL_d = lin_unstable_d.d;
% Controllability
rank(ctrb(ALU_d,BLU_d))
% Observability
rank(obsv(ALU_d,CL_d))

% Specs
zeta=0.7;ts=1.2;sigma=5/ts;wn=sigma/zeta;wd=wn*sqrt(1-zeta^2);
pole=-sigma+1i*wd;

% State feedback design
% Velocity control without integral action
A_co= [ALU(1:2,1:2) ALU(1:2,4);ALU(4,1:2) ALU(4,4)];
B_co= [BLU(1:2,1);BLU(4,1)];
C_co = [0 0 1]; % Select output that is controlled
P_co =[pole;conj(pole);real(pole)*5];
K_co = place(A_co,B_co,P_co);
N_co = -1/((C_co*inv(A_co - B_co*K_co)*B_co));
% Discrete
P_co_d = exp(P_co*Ts);
A_co_d = [ALU_d(1:2,1:2) ALU_d(1:2,4);ALU_d(4,1:2) ALU_d(4,4)];
B_co_d = [BLU_d(1:2,1);BLU_d(4,1)] ;
C_co_d = [0 0 1]; % Select output that is controlled
K_co_d = place(A_co_d, B_co_d, P_co_d);
N_co_d = 1/(C_co_d*inv(eye(3)-A_co_d+B_co_d*K_co_d)*B_co_d);


% Velocity control with integral action
% Continous
A_co_ia = [A_co zeros(3,1); C_co 0];
B_co_ia = [B_co ;0];
P_co_ia = [pole;conj(pole);real(pole)*5;real(pole)*6];
K_co_ia = place(A_co_ia,B_co_ia,P_co_ia );
K_co_ia_fb = K_co_ia(1:3);
K_co_ia_i = K_co_ia(4);
% Discrete
A_co_ia_d = [A_co_d zeros(3,1); C_co_d*Ts 1];
B_co_ia_d = [B_co_d ;0];
P_co_ia_d = exp(P_co_ia*Ts);
K_co_ia_d = place(A_co_ia_d, B_co_ia_d, P_co_ia_d);
K_co_ia_fb_d = K_co_ia_d(1:3);
K_co_ia_i_d = K_co_ia_d(4);



% Reduced order Luenberger observer in continuous−time
P_ro = 5* real(P_co(1));
% Reorder matrices
Ar11 = ALU(1:3,1:3); Ar12 = ALU(1:3,4);
Ar21 = ALU(4,1:3); Ar22 = ALU(4,4);
Br1 = BLU(1:3,1); Br2 = BLU(4,1);
Lr_ro = place(Ar22', Ar12', P_ro);
Lr_ro = Lr_ro';


% Reduced order Luenberger observer in discrete−time
P_ro_d = exp(P_ro * Ts );
% Reorder matrices
Ar11_d = ALU_d(1:3,1:3); Ar12_d = ALU_d(1:3,4);
Ar21_d = ALU_d(4,1:3); Ar22_d = ALU_d(4,4);
Br1_d = BLU_d(1:3,1); Br2_d = BLU_d(4,1);
Lr_ro_d = place(Ar22_d', Ar12_d', P_ro_d);
Lr_ro_d = Lr_ro_d';
A21_LA11=Ar21_d-Lr_ro_d*Ar11_d;
B2_LB1=Br2_d-Lr_ro_d*Br1_d;
A22_LA12=Ar22_d-Lr_ro_d*Ar12_d;


fcontent = fileread('rc_balance_team2_old_defs.h');
file=fopen('light_tracker6_defs.h','w');

fwrite(file, regexp(fcontent, '.*(?=\n.*?)', 'match', 'once'));
fprintf(file,'\n// Reduced order observator\n');
fprintf(file,'#define L                {%4.6f, %4.6f, %4.6f}\n',Lr_ro_d(1),Lr_ro_d(2),Lr_ro_d(3));
fprintf(file,'#define A21_LA11         {%4.6f, %4.6f, %4.6f}\n',A21_LA11(1),A21_LA11(2),A21_LA11(3));
fprintf(file,'#define B2_LB1           %4.6f\n',B2_LB1);
fprintf(file,'#define A22_LA12         %4.6f\n',A22_LA12);

fprintf(file,'\n// Velocity controller without intgral action\n');
fprintf(file,'#define N_co           %4.6f\n',N_co_d);
fprintf(file,'#define K_co         {%4.6f, %4.6f, %4.6f}\n',K_co_d(1),K_co_d(2),K_co_d(3));

fprintf(file,'\n#endif	// endif RC_BALANCE_CONFIG');
fclose(file);