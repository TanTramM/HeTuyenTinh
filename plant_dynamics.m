% =================================================================
% FINAL ATTEMPT: Corrected Equations + Total Mass Assumption
% =================================================================
function x_dot = plant_dynamics(x, tau)
% x: state vector input (6x1)
% tau: torque vector input (3x1)
% x_dot: state derivative output (6x1)

% --- 1. Deconstruct state vector for readability ---
th1 = x(1);
th2 = x(2);
th3 = x(3);
dth1 = x(4);
dth2 = x(5);
dth3 = x(6);

% --- 2. Physical Constants (from Table I) ---
m_total = 2.5; 
a1 = 0.25;     
a2 = 0.15;     
a3 = 0.15;     
g = 9.81;      

% *** FINAL MASS ASSUMPTION ***
% We are now testing the m_total assumption, as m/3 failed.
m1 = m_total;
m2 = m_total;
m3 = m_total;

% --- 3. Calculate Dynamics Matrices (M, V, G) ---
% These are the CORRECTED equations, not the paper's typos.
% Initialize matrices
M = zeros(3,3);
V = zeros(3,1);
G = zeros(3,1);

% --- Inertia Matrix M(theta) ---
% M(1,1) (Eq 17) - CORRECTED (m1 -> m2)
M(1,1) = (1/2)*m1*a1^2 + (1/2)*m2*a2^2 + m3*( (a2^2)*cos(th2)^2 + (1/3)*a3^2*cos(th2+th3)^2 + a2*a3*cos(th2+th3)*cos(th2) ) + (1/3)*m2*a2^2*cos(th2)^2;
M(1,2) = 0;
M(1,3) = 0;
M(2,1) = 0;
% M(2,2) (Eq 20) - Assumed correct as written
M(2,2) = (1/3)*a2^2*m2 + a2^2*m3 + (1/3)*a3^2*m3 + a2*a3*m3*cos(th3);
M(3,1) = 0;
% M(2,3) (Eq 21) - CORRECTED (Removed singular terms)
M(2,3) = (1/3)*a3^2*m3 + (1/2)*a2*a3*m3*cos(th3);
M(3,2) = M(2,3); % Symmetric
% M(3,3) (Eq 22) - Correct as written
M(3,3) = (1/3)*m3*a3^2;

% --- Coriolis/Centrifugal Vector V(theta, d_theta) ---
% V(1) (Eq 24) - CORRECTED (1/3 -> 4/3) and includes both terms
V1_term1 = ( -(4/3)*m2*a2^2*sin(2*th2) - (1/3)*m3*a3^2*sin(2*(th2+th3)) - m3*a2*a3*sin(2*th2+th3) ) * dth1 * dth2;
V1_term2 = ( -(1/3)*m3*a3^2*sin(2*(th2+th3)) - m3*a2*a3*cos(th2)*sin(th2+th3) ) * dth1 * dth3;
V(1) = V1_term1 + V1_term2; 

% V(2) (Eq 25) - Correct as written
V2_term1 = (-m3*a2*a3*sin(th3))*dth2*dth3 + (-1/2*m3*a2*a3*sin(th3))*dth3^2;
V2_term2 = ( (1/6)*m2*a2^2*sin(2*th2) + (1/6)*m3*a3^2*sin(2*(th2+th3)) + (1/2)*m3*a2^2*sin(2*th2) + (1/2)*m3*a2*a3*sin(2*th2+th3) ) * dth1^2;
V(2) = V2_term1 + V2_term2;

% V(3) (Eq 26) - Correct as written
V3_term1 = (1/2*m3*a2*a3*sin(th3))*dth2^2;
V3_term2 = ( (1/6)*m3*a3^2*sin(2*(th2+th3)) + (1/2)*m3*a2*a3*cos(th2)*sin(th2+th3) ) * dth1^2;
V(3) = V3_term1 + V3_term2;

% --- Gravity Vector G(theta) ---
G(1) = 0;
G(2) = (1/2)*m3*g*a3*cos(th2+th3) + (1/2)*m2*g*a2*cos(th2) + m3*g*a2*cos(th2);
G(3) = (1/2)*m3*g*a3*cos(th2+th3);

% --- 4. Calculate State Derivative ---
theta_ddot = M \ (tau - V - G); 

x_dot = [x(4); x(5); x(6); theta_ddot(1); theta_ddot(2); theta_ddot(3)];
end