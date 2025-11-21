%% --- SETUP_ROBOT.M ---
% Chỉ chứa điều kiện ban đầu và điểm đích
clear all;
clc;
disp('Đang nạp kịch bản mô phỏng...');

%% 1. ĐIỂM XUẤT PHÁT (Initial Conditions)
% Góc ban đầu (rad) - Từ Bảng IV bài báo
theta_initial = [0.7854; 
                -1.6280; 
                 1.6264];

% Vận tốc ban đầu (rad/s)
vel_initial = [0; 0; 0];

% Vector trạng thái ban đầu cho khối Integrator [Góc; Vận tốc]
X_initial = [theta_initial; vel_initial];

%% 2. ĐIỂM ĐÍCH (Setpoints)
% Góc mục tiêu (rad) - Từ Bảng IV bài báo
theta_final = [1.0304; 
              -0.3373; 
               0.3349];

disp('-> Đã nạp xong X_initial và theta_final.');
disp('-> Sẵn sàng chạy Simulink.');