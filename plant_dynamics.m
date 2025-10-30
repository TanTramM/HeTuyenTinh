% =================================================================
% TỆP 2: plant_dynamics.m
% HÀM NÀY SẼ ĐƯỢC GỌI BỞI KHỐI "MATLAB Function" TRONG SIMULINK
% =================================================================
function x_dot = plant_dynamics(x, tau)
% x: vector trạng thái đầu vào (6x1)
% tau: vector mô-men xoắn đầu vào (3x1)
% x_dot: vector đạo hàm trạng thái (6x1)

% --- 1. Tách các biến trạng thái cho dễ đọc ---
th1 = x(1);
th2 = x(2);
th3 = x(3);
dth1 = x(4);
dth2 = x(5);
dth3 = x(6);

% --- 2. Các hằng số vật lý (Từ Bảng I) ---
% Lấy từ Bảng I [cite: 431]
m_total = 2.5; % Tổng khối lượng (Kg)
a1 = 0.25;     % Chiều dài link 1 (m)
a2 = 0.15;     % Chiều dài link 2 (m)
a3 = 0.15;     % Chiều dài link 3 (m)
g = 9.81;      % Gia tốc trọng trường (m/s^2)

% **GHI CHÚ QUAN TRỌNG VỀ KHỐI LƯỢNG**
% Bài báo dùng m1, m2, m3 trong phương trình (17)-(30) [cite: 523-593]
% nhưng Bảng I chỉ ghi "m = 2.5 Kg"[cite: 431].
% Đây là một điểm không rõ ràng. Chúng ta GIẢ ĐỊNH là khối lượng
% được chia đều cho 3 link.
m1 = m_total / 3;
m2 = m_total / 3;
m3 = m_total / 3;

% --- 3. Lập trình các phương trình Động lực học (M, V, G) ---
% Dựa trên các phương trình (16) đến (30) của bài báo.

% Khởi tạo Ma trận M, V, G
M = zeros(3,3);
V = zeros(3,1);
G = zeros(3,1);

% Ma trận Quán tính M(theta)
M(1,1) = (1/2)*m1*a1^2 + (1/2)*m2*a2^2 + m3*( (a2^2)*cos(th2)^2 + (1/3)*a3^2*cos(th2+th3)^2 + a2*a3*cos(th2+th3)*cos(th2) ) + (1/3)*m2*a2^2*cos(th2)^2; 
M(1,2) = 0; 
M(1,3) = 0; 
M(2,1) = 0; 
M(2,2) = (1/3)*a2^2*m2 + a2^2*m3 + (1/3)*a3^2*m3 + a2*a3*m3*cos(th3); 
M(2,3) = (1/3)*a3^2*m3 + a2^2*m3 + (1/3)*a2*a3*m3*cos(th3); 
M(3,1) = 0; 
M(3,2) = M(2,3); % Ma trận M đối xứng 
M(3,3) = (1/3)*m3*a3^2; 

% Véc-tơ Corriolis/Ly tâm V(theta, d_theta)
V1_term1 = ( -(1/3)*m2*a2^2*sin(2*th2) - (1/3)*m3*a3^2*sin(2*(th2+th3)) - m3*a2*a3*sin(2*th2+th3) ) * dth1 * dth2;
V1_term2 = ( -(1/3)*m3*a3^2*sin(2*(th2+th3)) - m3*a2*a3*cos(th2)*sin(th2+th3) ) * dth1 * dth3;
V(1) = V1_term1 + V1_term2;

V2_term1 = (-m3*a2*a3*sin(th3))*dth2*dth3 + (-1/2*m3*a2*a3*sin(th3))*dth3^2; 
V2_term2 = ( (1/6)*m2*a2^2*sin(2*th2) + (1/6)*m3*a3^2*sin(2*(th2+th3)) + (1/2)*m3*a2^2*sin(2*th2) + (1/2)*m3*a2*a3*sin(2*th2+th3) ) * dth1^2; 
V(2) = V2_term1 + V2_term2;

V3_term1 = (1/2*m3*a2*a3*sin(th3))*dth2^2; 
V3_term2 = ( (1/6)*m3*a3^2*sin(2*(th2+th3)) + (1/2)*m3*a2*a3*cos(th2)*sin(th2+th3) ) * dth1^2;
V(3) = V3_term1 + V3_term2;

% Véc-tơ Trọng lực G(theta)
G(1) = 0; 
G(2) = (1/2)*m3*g*a3*cos(th2+th3) + (1/2)*m2*g*a2*cos(th2) + m3*g*a2*cos(th2); 
G(3) = (1/2)*m3*g*a3*cos(th2+th3); 

% --- 4. Tính toán Đạo hàm Trạng thái ---
% Phương trình động lực học: M(th)*th_ddot + V(th,th_dot) + G(th) = tau
% => th_ddot = inv(M) * (tau - V - G)
theta_ddot = M \ (tau - V - G); % Dùng M\ thay cho inv(M)*

% Kết quả cuối cùng x_dot = [d_theta; dd_theta]
x_dot = [x(4); x(5); x(6); theta_ddot(1); theta_ddot(2); theta_ddot(3)];
end