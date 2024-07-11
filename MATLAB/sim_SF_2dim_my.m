clc
clear
close all

%アニメーション設定
anime_ON = 0;  % アニメーションの　オン・オフ 0:なし，1:ひとつ，2:いろんな角度
SS = 5;   % アニメーション速度，プロットの間隔
PPG_size = 0.5;   % アニメーションの時のPPGの大きさ
% アニメーションの視点
az_a = -37.5;   %方位角(3次元初期設定は -37.5)
el_a = 30;   %仰角(3次元初期設定は 30)
plot2D = 0;   % 2次元プロットの有無 

%シミュレーション時間
dt = 0.05; 
t_end = 100;
%t_end = 55;
time =0:dt:t_end;

%初期状態：慣性座標系における目標座標からの偏差
x_eI_0 = [0,0,0*pi/180]; %x_0,y_0,psi_0 %rem


%UAV速度設定
V = 12;  %対気速度[m/s]

%初期状態の計算
Cxi0 = [0, 0, 0, dt, 0, 0, 0]; % F_PATH へ与える入力の初期状態:s,dot_s,t,dt,s_old,xi_old,i_sar
X_I_0 = F_PATH_FX79_r1(Cxi0)'+[x_eI_0, 0, 0, 0, 0]'; % 慣性座標系における初期状態：x_d,y_d,χ_d,dχ_d,κ,xi,i

%事前割り当て
x_e_vec = zeros([length(time),5]); % 誤差ダイナミクスの状態量x_eベクトル，および s, xi
dx_e_vec = zeros([length(time),5]);% x_eベクトルの微分値
F = zeros([length(time),7]); % 慣性座標系における"目標"座標，および曲率など
X_I = zeros([length(time),5]); % 慣性座標位置における状態，および s
dX_I = zeros([length(time),5]);% 慣性座標位置における状態の微分値
V_g = zeros([length(time),1]); % 対地速度v_g
phi = zeros([length(time),1]); % ロール角Φ
p = zeros([length(time),1]); % ロール角速度p
phi_r = zeros([length(time),1]); % 目標ロール角Φr
dphi_r = zeros([length(time),1]); % ロール角速度dΦr
delta_a = zeros([length(time),1]); % エルロン入力
error = zeros([length(time),1]); % ヨー角と飛行経路角の差分の観測用
GammaChi = zeros([length(time),2]);
Cxi = zeros([length(time),7]);   % F_PATHへの入力の計算用
ZV = zeros([length(time),1]); % 0ベクトル

phi_r_f= zeros([length(time),1]);
D_phi_r_f = zeros([length(time),1]);

dchi_d = zeros([length(time),1]);

dchi_d_f = zeros([length(time),1]);
D_dchi_d_f = zeros([length(time),1]);

delta_a_x = zeros([length(time),1]);

% 初期値の代入
X_I(1,:) = X_I_0(1:5,1)';   % 初期値の代入
Cxi(1,:) = Cxi0;

phi_0 = 0;
p_0 = 0;

phi(1,1) = phi_0;
p(1,1) = p_0;

%% 計算
for i = 1 : length(time)
    
    %時間を表示
    t = i*dt
    
    %風の設定
    Wx = 3*sin(2*t);
    Wy = 3*sin(2*t);
    
    % Wx = 1;
    % Wy = 1;

    %慣性座標系における状態の取り出し．
    xI =X_I(i,1); %位置x[m]
    yI =X_I(i,2); %位置y[m]
    psi=X_I(i,3); %ヨー角[rad]
    s = X_I(i,4); %経路長s [m]

end