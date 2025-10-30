% =================================================================
% TỆP 1: setup_simulation.m
% CHẠY TỆP NÀY TRƯỚC KHI CHẠY SIMULINK
% =================================================================
clear; clc;
disp('Đang khởi tạo các biến mô phỏng từ bài báo LQR...');

% --- 1. Thông số Điều khiển (Trụ cột 2) ---

% Ma trận K (Full State Feedback Gain)
% Lấy trực tiếp từ Phương trình (44).
K = [100.00    0         0    325.32    0         0;
       0    100.00     0       0    316.35    102.11;
       0       0    100.00     0    102.11    112.14];

% --- 2. Thông số Mô phỏng (Từ Bảng IV) ---
% Lấy các giá trị ban đầu và mục tiêu từ Bảng IV.

% Trạng thái ban đầu x(0) (vector 6x1)
% X = [theta1, theta2, theta3, d_theta1, d_theta2, d_theta3]'
theta_initial = [0.7854; -1.6280; 1.6264];
theta_dot_initial = [0; 0; 0];
x0 = [theta_initial; theta_dot_initial];

% Tín hiệu tham chiếu (Reference) (vector 6x1)
% Đây là đầu vào R (Reference Input) trong Hình 6.
theta_final = [1.0304; -0.3373; 0.3349];
theta_dot_final = [0; 0; 0];
R_ref = [theta_final; theta_dot_final];

disp('Hoàn tất! Các biến K, x0, và R_ref đã sẵn sàng.');