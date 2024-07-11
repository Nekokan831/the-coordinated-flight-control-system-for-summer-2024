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
V_a = 12;  %対気速度[m/s]

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

%定数
g=9.80665;
rho=1.155; %ρ
Ss=0.42;
ww=2; 
d0=0.2393;     %δe^0
Ixx=0.1580;    

Mc=rho*Ss*ww*V^2*(0.1659*d0+0.2578)/2;

%% パラメータ
a = 40;
b = 0.8;
c = 0;

%% ゲイン
fp = 1.7;
fd = 0.15;

%% 計算（シミュレーション本体）
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

    %参照経路情報(現時点の経路情報)
    F(i,:) = F_PATH_FX79_r1(Cxi(i,:))';

    %参照経路情報(現時点の経路情報)の取り出し
    chi_d = F(i,3);  % 目標航路角[rad]
    kappa = F(i,5);  % 曲率[rad/m]
    xi = F(i,6);     % 媒介変数 xi（ζ）

    %慣性座標系におけるx,yの目標との偏差
    x_eI = (X_I(i,1:2) - F(i,1:2))';

    %対地速度Vgの計算
    %機体座標系{B} → 慣性座標系{I}の回転行列...?個人的な解釈は，V_aをベクトルにするための三角関数
     SyIB=[cos(psi) sin(psi);  % ヨー回転
          -sin(psi) cos(psi)];

    % 対地速度ベクトル,x方向はそのままdot_x, y方向はそのままdot_yとなる
    V_g_vec = SyIB*[V_a; 0] + [Wx; Wy];  % [m/s]

    % 対地速度の大きさ（ノルム）
    V_g(i,1) = norm(V_g_vec);  % [m/s]

    % 航路角χの導出
    chi = atan2(-V_g_vec(2),V_g_vec(1));  % [rad]

    %% 慣性座標系からセレ・フレネ座標系への変換
    %χeの補正
    x_e_vec(i,3) = -chi + chi_d; %χeの導出
                                 %chi - chi_dではない理由は，回転行列のsinの符号を揃えるため...?
    while (x_e_vec(i,3) < -pi || x_e_vec(i,3) >= pi)  % 0 <= chi_e < 2*pi の否定
        if (x_e_vec(i,3) > pi)
            x_e_vec(i,3) = x_e_vec(i,3) - 2*pi;
        elseif (x_e_vec(i,3) < -pi)
            x_e_vec(i,3) = x_e_vec(i,3) + 2*pi;
        end
    end

    GammaChi(i,1) = chi;  % 航路角χの１ステップ前の保存用

    %慣性座標系{I}→セレ・フレネ座標系{F}の回転行列
    %ここはzの座標軸を機体下向きに取っているのでsinの符号が普通の機体とは逆．発狂．
    SyIF0=[cos(chi_d) -sin(chi_d);
           sin(chi_d) cos(chi_d)];

    %慣性座標系におけるx,y偏差を，セレ・フレネ座標系における誤差xe,yeへ座標軸回転
    x_e_vec(i,1:2) = (SyIF0*x_eI)';

    x_e_vec(i,4) = s;   % s の代入
    x_e_vec(i,5) = xi;   % xi(s)の値を取得

    %セレ・フレネ座標系における変数x_e_vecについて再定義．
    x_e = x_e_vec(i,1);  % [m]
    y_e = x_e_vec(i,2);  % [m]
    chi_e = x_e_vec(i,3);  % [rad]



    %% 目標ロール角の計算
    %目標ロール角の計算
    %A_Practical_Design_Approach_for_Complex_Path_Tracking_Controlの式26
    phi_r(i,1) = atan((V_g(i,1)/(a*g))*(b*(y_e+a*chi_e)+V_g(i,1)*sin(chi_e)-V_g(i,1)*kappa*x_e*cos(chi_e)+a*dchi_d(i,1)-c*kappa*x_e^2));
    
end