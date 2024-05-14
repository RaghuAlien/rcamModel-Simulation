function [X_dot] = rcam_model(X,U)
 
%-------------STATE & CONTROL VECTORS---------------%

% EXTRACT STATE VECTORS

x1 = X(1); % u
x2 = X(2); % v
x3 = X(3); % w
x4 = X(4); % p
x5 = X(5); % q
x6 = X(6); % r
x7 = X(7); % phi
x8 = X(8); % theta
x9 = X(9); % psi

u1 = U(1); % delta(Alieron)
u2 = U(2); % delta(Elevator--> Horizontal tail)
u3 = U(3); % delta(rudder)
u4 = U(4); % delta(throttle_Engine1)
u5 = U(5); % delta(throttle_Engine2)

%--------------------------CONSTANTS-----------------------------------
% VEHICLE CONSTANTS - all are in SI units

m = 120000;                 % mass

% INERTIA MATRIX
I_b = (1/m)*[40.07 0 -2.0923;
            0 64 0;
            -2.0923 0 9.92];
invI_b = (1/m) * [0.0249836 0 0.000523151;
                0 0.015625 0;
                0.000523151 0 0.010019];

% NOTE: inv(I) is constant!, 
% Hardcoded because saving compting time, when this finction running
% for n-number of time

cbar = 6.6;                 % mean aerodynamic chord
lt =  24.8;                 % ac of tail to ac of wing
S = 260;                    % wing platform area 
St = 64;                    % tail platform area

Xcg = 0.23*cbar;            % X position of CoG
Ycg = 0;                    % Y position of CoG
Zcg = 0.1*cbar;             % Z position of CoG

Xac = 0.12*cbar;            % X position of AC
Yac = 0;                    % Y position of AC
Zac = 0;                    % Z position of AC

% ENGINE CONSTANTS

Xapt1 = 0;                  % X position of Engine 1 APT- Application of Thrust
Yapt1 = -7.94;              % Y position of Engine 1
Zapt1 = -1.9;               % Z position of Engine 1

Xapt2 = 0;                  % X position of Engine 2
Yapt2 = 7.94;               % Y position of Engine 2
Zapt2 = -1.9;               % Z position of Engine 2

% OTHER CONSTANTS

rho = 1.225;                % air density
g = 9.81;                   % gravity
d_epsd_a = 0.25;            % d_epsilon/d_alpha
alpha_L0 = -11.5*pi/180;    % zero lift AoA
a = 5.5;                    % Linear Lift curve slop
a3 = -768.5;                % Lift curve slop alpha^3
a2 = 609.2;                 % Lift curve slop alpha^2
a1 = 155.2;                 % Lift curve slop alpha^1
a0 = 15.212;                % Lift curve slop alpha^0
alpha_switch = 14.5*pi/180; % alpha where lift curve changes from linear to non-linear

%---------------- 1. Control Limits--------------------------

% Implemented in Simulink iteself

%---------------------2. INTERMEDIATE VALUES --------------------------
% AIRSPEED
Va = sqrt(x1^2+x2^2+x3^2);

% ANGLES
alpha = atan2(x3,x1);
beta = asin(x2/Va);

% DYNAMIC PRESSURE
Q = 0.5 * rho * Va^2;

% Defining Linear & angluar velocity w.r.t earth on body V_b & wbe_b
V_b = [x1;x2;x3];
wbe_b = [x4;x5;x6];


% --------- 3. NON-DIMENSINAL AERO CO-EFFICIENTS IN STABILITY FRAME--------
% CALCULATE CL_wb
if alpha < alpha_switch
    CL_wb = a  * (alpha - alpha_L0);
else
    CL_wb = a3*alpha^3 + a2*alpha^2 + a1*alpha + a0;
end

% CALCULATE CL_t
epsilon = d_epsd_a * (alpha-alpha_L0);
alpha_t = alpha - epsilon + u2 + 1.3*x5*lt/Va;
CL_t = 3.1*St/S*alpha_t;

% TOTAL LIFT
CL = CL_wb + CL_t;

% TOTAL DRAG
CD = 0.13 + 0.07*(5.5*alpha + 0.654)^2;

% SIDE FORCE
CY = -1.6*beta + 0.24*u3;

% ----------------4. DIMENSIONAL AERO FORCES IN BODY FRAME------------

FA_s = [-CD*Q*S; 
        CY*Q*S;
        -CL*Q*S];           % Force vector in Stabilty Frame
  
R_bs = [cos(alpha) 0 -sin(alpha);
        0 1 0;
        sin(alpha) 0 cos(alpha)];

FA_b = R_bs*FA_s;           % Force vector in Body Frame

% -------------5. NON-DIMENSIONAL AERO MOMENT COEFF ABOUT AERO_CENTER------
% CALCULATE MOMENT COEFF IN BODY FRAME

eta1 = -1.4*beta;
eta2 = -0.59 - (3.1*(St*lt)/(S*cbar)) * (alpha-epsilon);
eta3 = (1 - alpha*(180/15*pi)) * beta;

eta = [eta1;eta2;eta3];

dCMdx = cbar/Va * [-11 0 5; 
                    0 (-4.03-(St*lt^2)/(S*cbar^2)) 0;
                    1.7 0 -11.5];   % Moment change with respect to states

dCMdu = [-0.6 0 0.22;
         0 (-3.1*(St*lt)/(S*cbar)) 0;
         0 0 -0.63];                % Moment chnage w.r.t to control

% Coeff of Aero moment about AC in body frame [Cl;Cm;Cn]
CMac_b = eta + dCMdx*wbe_b + dCMdu * [u1;u2;u3];


%-------------- 6. AERO MOMENT ABOUT AC ---------------------------

MAac_b = CMac_b * Q * S * cbar;

% ---------------7. AERO MOMENT ABOUT CG ------------------------
% MOMENT TRANSFER
rcg_b = [Xcg;Ycg;Zcg];
rac_b = [Xac;Yac;Zac];

MAcg_b = MAac_b + cross(FA_b,rcg_b-rac_b);

% --------------- 8. ENGINE FORCE & MOMENT ------------------------
% THRUST ON EACH ENGINE
F1 = u4*m*g;
F2 = u5*m*g;

% FORCE VECTOR 
FE1_b = [F1;0;0];
FE2_b = [F2;0;0];
FE_b = FE1_b+FE2_b;

% MOMENT CALCULATION beacuse engine offset from CoG
mew1 = [Xcg-Xapt1;
        Yapt1 - Ycg;
        Zcg - Zapt1];

mew2 = [Xcg-Xapt2;
        Yapt2 - Ycg;
        Zcg - Zapt2];
MEcg1_b = cross(mew1,FE1_b);
MEcg2_b = cross(mew2,FE2_b);

MEcg_b = MEcg1_b + MEcg2_b;


% -------------------9. Gravity Force & Moment ------------------------

g_b = [-g*sin(x8);
        g*cos(x8)*sin(x7)
        g*cos(x8)*cos(x7)];

Fg_b = m*g_b;

% ----------------- 10. STATE DERIVATIVES ------------------------------

% CALCULATE u_dot,v_dot,w_dot from F_b
F_b = FA_b + FE_b + Fg_b;
x1to3_dot = (1/m)*F_b - cross(wbe_b,V_b);

% CALCULATE p_dot,q_dot,r_dot from Mac_b
Mcg_b = MAcg_b + MEcg_b;
x4to6_dot = invI_b*(Mcg_b - cross(wbe_b,(I_b*wbe_b)));

% CALCULATE phi_dot, theta_dot, psi_dot
R = [1 sin(x7)*tan(x8) cos(x7)*tan(x8);
    0 cos(x7) -sin(x7);
    0 sin(x7)/cos(x8) cos(x7)/cos(x8)];

x7to9 = R*wbe_b;

% ASSMEBLY

X_dot = [x1to3_dot; x4to6_dot; x7to9]; 

