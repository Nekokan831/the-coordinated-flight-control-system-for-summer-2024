% sim_2D_PPG_r11.m
% 2022/3/31 by Yutoku Takahashi
% 2次元モデル　協調制御
% 4ルール

% 論文投稿バージョン
% データ整理
%% _/_/_/_/_/_/  注意!!  _/_/_/_/_/_/
% 関数 F_PATH_PPG_r** に目標経路を記述すること!!
%_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

%% 文字設定と初期化
% 初期化
clc
clear
close all

% フォントと文字サイズの設定
% set(0,'defaultAxesFontSize',15)   %初期値は 9
set(0,'defaultTextFontSize',15)   %初期値は 9

%% パラメータ，目標経路などの設定
% % % % シミュレーション・アニメーション環境設定 % % % %
% シミュレーション時間設定
dt = 0.1;   % シミュレーション 制御周期
% time = 0:dt:80;   % シミュレーション時間
% time = 0:dt:0.1;   % シミュレーション時間
time = 0:dt:360;   % シミュレーション時間
% time = 0:dt:80;   % シミュレーション時間

% アニメーション設定
anime_ON = 0;   % アニメーションの　オン・オフ 0:なし，1:ひとつ，2:いろんな角度
SS = 5;   % アニメーション速度，プロットの間隔
PPG_size = 0.5;   % アニメーションの時のPPGの大きさ

fig_num = 8; % グラフの枚数

% アニメーションの視点
% az_a = -37.5;   %方位角(3次元初期設定は -37.5)
% el_a = 30;   %仰角(3次元初期設定は 30)
az_a = 0;   %方位角(3次元初期設定は -37.5)
el_a = 90;   %仰角(3次元初期設定は 30)

% 色設定ベクトル
CV = [0 0 1; % 青
    1 0 0; % 赤
    0.9290 0.6940 0.1250; % オレンジ
    0 0 0; % 黒
    0 1 1]; % 水色 

% % % % UAV初期状態・風外乱設定 % % % %
% 慣性座標系における目標座標からの偏差
x_I_0{1} = [-50, -50, 90*pi/180]; % x_0, y_0, psi_0
x_I_0{2} = [-50, 50, -90*pi/180]; % x_0, y_0, psi_0
x_I_0{3} = [50, -50, 0*pi/180]; % x_0, y_0, psi_0
x_I_0{4} = [50, 50, 180*pi/180]; % x_0, y_0, psi_0
x_I_0{5} = [-50, 0, 90*pi/180]; % x_0, y_0, psi_0

% x_I_0{1} = [300, 100, 0.45*pi]; % x_0, y_0, psi_0
% x_I_0{2} = [200, 100, 0.45*pi]; % x_0, y_0, psi_0
% x_I_0{3} = [100, 100, 0.45*pi]; % x_0, y_0, psi_0
% x_I_0{4} = [  0, 100, 0.45*pi]; % x_0, y_0, psi_0

% x_I_0{1} = [  0, 100, 0.45*pi]; % x_0, y_0, psi_0
% x_I_0{2} = [100, 100, 0.45*pi]; % x_0, y_0, psi_0
% x_I_0{3} = [200, 100, 0.45*pi]; % x_0, y_0, psi_0
% x_I_0{4} = [300, 100, 0.45*pi]; % x_0, y_0, psi_0

% x_I_0{1} = [280, 5, -0.5*pi]; % x_0, y_0, psi_0
% x_I_0{2} = [280, 15, -0.55*pi]; % x_0, y_0, psi_0
% x_I_0{3} = [280, 25, -0.6*pi]; % x_0, y_0, psi_0
% x_I_0{4} = [280, 35, -0.65*pi]; % x_0, y_0, psi_0

% x_I_0{1} = [-50, -50, 0*pi/180]; % x_0, y_0, psi_0
% x_I_0{2} = [-50, 50, 0*pi/180]; % x_0, y_0, psi_0
% x_I_0{3} = [50, -50, 0*pi/180]; % x_0, y_0, psi_0
% x_I_0{4} = [50, 50, 0*pi/180]; % x_0, y_0, psi_0
% x_I_0{5} = [-50, 0, 0*pi/180]; % x_0, y_0, psi_0


% x_I_0{1} = [235,100, -0.6*pi]; % x_0, y_0, psi_0
% % % % x_I_0{1} = [235,100, -80*pi/180]; % x_0, y_0, psi_0
% x_I_0{2} = [285, 0, -0.52*pi]; % x_0, y_0, psi_0
% x_I_0{3} = [212, -140, -0.2*pi]; % x_0, y_0, psi_0
% % x_I_0{4} = [110, -200, -0.05*pi]; % x_0, y_0, psi_0
% x_I_0{4} = [110, -200, -0.5*pi]; % x_0, y_0, psi_0

% x_I_0{1} = [295, 0, -0.52*pi]; % x_0, y_0, psi_0
% x_I_0{2} = [271, 2, -0.52*pi]; % x_0, y_0, psi_0
% x_I_0{3} = [283, 1, -0.52*pi]; % x_0, y_0, psi_0
% x_I_0{4} = [284, 13, -0.52*pi]; % x_0, y_0, psi_0

% % % % フィードバックゲインゲイン設定 % % % %
% % コスト保証，入力制限なし
% Fg{1} = [-0.0713   -0.0000   -0.0006         0
%    -0.0040    0.0090    0.2921         0
%          0         0         0    0.0507];
% Fg{2} = [-0.0713   -0.0000   -0.0006         0
%    -0.0041    0.0010    0.2624         0
%          0         0         0    0.0507];
% Fg{3} = [-0.0713   -0.0000   -0.0006         0
%     0.0040    0.0090    0.2922         0
%          0         0         0    0.0507];
% Fg{4} = [-0.0713   -0.0000   -0.0006         0
%     0.0041    0.0010    0.2624         0
%          0         0         0    0.0507];

% コスト保証，入力制限あり
Fg{1} = [-0.1102   -0.0006   -0.1382         0
   -0.0049    0.0091    0.2953         0
         0         0         0    0.0105];
Fg{2} = [-0.1102   -0.0006   -0.1382         0
   -0.0048    0.0012    0.2780         0
         0         0         0    0.0105];
Fg{3} = [-0.1102   -0.0006   -0.1382         0
    0.0048    0.0091    0.2951         0
         0         0         0    0.0105];
Fg{4} = [-0.1102   -0.0006   -0.1382         0
    0.0047    0.0012    0.2781         0
         0         0         0    0.0105];

% Fg{1} = [];
% Fg{2} = [];
% Fg{3} = [];
% Fg{4} = [];

% Fg{1} = [];
% Fg{2} = [];
% Fg{3} = [];
% Fg{4} = [];

% Fg{1} = [];
% Fg{2} = [];
% Fg{3} = [];
% Fg{4} = [];

% Fg{1} = [];
% Fg{2} = [];
% Fg{3} = [];
% Fg{4} = [];

% Fg{1} = [];
% Fg{2} = [];
% Fg{3} = [];
% Fg{4} = [];

% Fg{1} = [];
% Fg{2} = [];
% Fg{3} = [];
% Fg{4} = [];

% 参考文献で使用しているUAV用のゲイン
% Fg{1} = [-0.0678   -0.0000   -0.0010         0
%    -0.0044    0.0097    0.4468         0
%          0         0         0    0.0505];
% Fg{2} = [-0.0678   -0.0000   -0.0010         0
%    -0.0045    0.0008    0.4168         0
%          0         0         0    0.0505];
% Fg{3} = [-0.0678   -0.0000   -0.0010         0
%     0.0044    0.0097    0.4471         0
%          0         0         0    0.0505];
% Fg{4} = [-0.0678   -0.0000   -0.0010         0
%     0.0045    0.0008    0.4168         0
%          0         0         0    0.0505];


% Fg{1} = [-0.0745   -0.0000   -0.0000         0
%    -0.0044    0.0097    0.3979         0
%          0         0         0    0.0501];
% Fg{2} = [-0.0745   -0.0000   -0.0000         0
%    -0.0047    0.0006    0.4536         0
%          0         0         0    0.0501];
% Fg{3} = [-0.0745   -0.0000   -0.0000         0
%     0.0044    0.0097    0.3979         0
%          0         0         0    0.0501];
% Fg{4} = [-0.0745   -0.0000   -0.0000         0
%     0.0047    0.0006    0.4536         0
%          0         0         0    0.0501];

% Fg{1} = [-0.4241   -0.0000   -0.0000         0
%    -0.0031    0.0103    0.4545         0
%          0         0         0    0.0616];
% Fg{2} = [-0.4241   -0.0000   -0.0000         0
%    -0.0031    0.0007    0.5640         0
%          0         0         0    0.0616];
% Fg{3} = [-0.4241   -0.0000   -0.0000         0
%     0.0031    0.0103    0.4545         0
%          0         0         0    0.0616];
% Fg{4} = [-0.4241   -0.0000   -0.0000         0
%     0.0031    0.0007    0.5640         0
%          0         0         0    0.0616];

% Fg{1} = [];
% Fg{2} = [];
% Fg{3} = [];
% Fg{4} = [];

% Fg{1} = [];
% Fg{2} = [];
% Fg{3} = [];
% Fg{4} = [];

% % % % UAVの設定 % % % %
V0 = 15;   % 対気速度
V_min=5;     % 機体速度 最小値[m/s]
V_max=25;     % 機体速度 最大値[m/s]
% V_min=10;     % 機体速度 最小値[m/s]
% V_max=20;     % 機体速度 最大値[m/s]
mu_x = 15;   % 入力制限

% V0 = 28;   % 対気速度
% V0 = 35;   % 対気速度
% V_min=20;     % 機体速度 最小値[m/s]
% V_max=50;     % 機体速度 最大値[m/s]
% mu_x = 45;   % 入力制限

k_max = 1.3;
k_min = 0.7;

% ファジィ化の範囲
epsilon1 = sin(178*pi/180) / (178*pi/180);
kappa_max = 0.05;
D_kappa_max = (V_max+mu_x)*kappa_max;    % 機体速度 最大値[m/s]
D_kappa_min = -D_kappa_max;

% 風 の設定
% Wx = 1/2^(1/2);
% Wy = -1/2^(1/2);
% (Wx^2+Wy^2)^(1/2)
Wx = 1;
Wy = 1;
% Wx = 12.5^(1/2);
% Wy = -12.5^(1/2);
% Wx = 8;
% Wx = 8;
% Wy = 0;

% 目標経路に関する設定
time_c1=120; % 切り替え時間１
time_c2=240; % 切り替え時間２

eta1{1} = 1;
eta1{2} = 1.1;
eta1{3} = 1.2;
eta1{4} = 0.9;
eta1{5} = 0.8;

% eta1{1} = 1;
% eta1{2} = 1;
% eta1{3} = 1;
% eta1{4} = 1;
% eta1{5} = 1;

eta2{1} = 1;
eta2{2} = 1;
eta2{3} = 1;
eta2{4} = 1;
eta2{5} = 1;

eta3{1} = 1;
eta3{2} = 1.1;
eta3{3} = 1.2;
eta3{4} = 0.9;
eta3{5} = 0.8;

s_0_1{1}=0;
s_0_1{2}=0;
s_0_1{3}=0;
s_0_1{4}=0;
s_0_1{5}=0;

% s_0_1{1}=0;
% s_0_1{2}=10;
% s_0_1{3}=20;
% s_0_1{4}=10;
% s_0_1{5}=20;
% 
% s_0_1{1}=0;
% s_0_1{2}=20;
% s_0_1{3}=40;
% s_0_1{4}=60;
% s_0_1{5}=80;

% s_0_1{1}=0;
% s_0_1{2}=30;
% s_0_1{3}=60;
% s_0_1{4}=90;
% s_0_1{5}=120;

% s_0_1{1}=-80;
% s_0_1{2}=-60;
% s_0_1{3}=-40;
% s_0_1{4}=-20;
% s_0_1{5}=0;

% s_0_1{1}=114.2186;
% s_0_1{2}=29.9760;
% s_0_1{3}=-103.0499;
% s_0_1{4}=-199.1559;
% s_0_1{5}=80;

s_0_2{1}=0;
s_0_2{2}=10;
s_0_2{3}=20;
s_0_2{4}=30;
s_0_2{5}=40;

s_0_3{1}=0;
s_0_3{2}=10;
s_0_3{3}=20;
s_0_3{4}=10;
s_0_3{5}=20;

% error0{1}=114.2186;
% error0{2}=29.9760;
% error0{3}=-103.0499;
% error0{4}=-199.1559;

%% 以下基本的に全自動

%% 初期状態の計算
eta{1} = eta1{1};
eta{2} = eta1{2};
eta{3} = eta1{3};
eta{4} = eta1{4};
eta{5} = eta1{5};
s0{1} = s_0_1{1};
s0{2} = s_0_1{2};
s0{3} = s_0_1{3};
s0{4} = s_0_1{4};
s0{5} = s_0_1{5};

num_UAV = length(x_I_0); % UAVの台数
for iu = 1:num_UAV
    Cxi0{iu} = [s0{iu}, 0, 0, dt, s0{iu}, 0, 0, iu, eta{iu},s0{iu}]; % F_PATH へ与える入力の初期状態
    X_I_0{iu} = F_PATH_PPG_r3(Cxi0{iu})'+[x_I_0{iu}, s0{iu}, 0, 0, 0]'; % 慣性座標系における初期状態
%     X_I_0{iu} = [x_I_0{iu}, s0{iu}, 0, 0, 0]'; % 慣性座標系における初期状態
%     X_I_0{iu} = [x_I_0{iu}, 30*(iu-1), 0, 0, 0]'; % 慣性座標系における初期状態
%     X_I_0{iu} = [x_I_0{iu}, error0{iu}, 0, 0, 0]'; % 慣性座標系における初期状態
%     X_I_0{iu} = [x_I_0{iu}, 0, 0, 0, 0]'; % 慣性座標系における初期状態
end

%% 事前割り当て
% 初期位置，初期速度
for iu = 1:num_UAV
    x_e_vec{iu} = zeros([length(time),5]); % 誤差ダイナミクスの状態量x_eベクトル，および s, xi
    dx_e_vec{iu} = zeros([length(time),6]);% x_eベクトルの微分値
    F{iu} = zeros([length(time),7]); % 慣性座標系における目標座標，および曲率など
    X_I{iu} = zeros([length(time),5]); % 慣性座標位置における状態，および s
    dX_I{iu} = zeros([length(time),5]);% 慣性座標位置における状態の微分値
    M1{iu} = zeros([length(time),1]);
    M2{iu} = zeros([length(time),1]);
    N1{iu} = zeros([length(time),1]);
    N2{iu} = zeros([length(time),1]);
    u_vec{iu} = zeros([length(time),3]); % 入力量Uベクトル
    V{iu} = zeros([length(time),1]); % 対気速度 v
    Vg{iu} = zeros([length(time),1]); % 対地速度 v_g
    error{iu} = zeros([length(time),1]); % ヨー角と飛行経路角の差分の観測用
    Chi_save{iu} = zeros([length(time),2]);
    Cxi{iu} = zeros([length(time),10]);   % F_PATHへの入力の計算用
    Cxi{iu}(1,2)=V0;
    % 初期状態
    X_I{iu}(1,:) = X_I_0{iu}(1:5,1)';   % 初期値の代入
    Cxi{iu}(1,:) = Cxi0{iu};
    J_sum{iu}=0;
end
ZV = zeros([length(time),1]); % 0ベクトル

%% 入力量，目標軌道や慣性座標位置の計算
for i = 1:length(time)
    % 時間を表示
    t = (i-1)*dt

% etaの途中切り替え処理（論文には使用せず）%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if t < time_c1
        eta{1} = eta1{1};
        eta{2} = eta1{2};
        eta{3} = eta1{3};
        eta{4} = eta1{4};
        eta{5} = eta1{5};
    elseif time_c1 <= t && t < time_c2
        eta{1} = eta2{1};
        eta{2} = eta2{2};
        eta{3} = eta2{3};
        eta{4} = eta2{4};
        eta{5} = eta2{5};
        if t == time_c1
            for iu = 1:num_UAV
                X_I{iu}(i,4) = s_0_2{iu}-s_0_1{iu};
                s0{iu} = s_0_2{iu};
            end
        end
    else
        eta{1} = eta3{1};
        eta{2} = eta3{2};
        eta{3} = eta3{3};
        eta{4} = eta3{4};
        eta{5} = eta3{5};
        if t == time_c2
            for iu = 1:num_UAV
                X_I{iu}(i,4) = s_0_3{iu}-s_0_2{iu};
                s0{iu} = s_0_3{iu};
            end
        end
    end
    
% Step1: x,y,chi,s,Deltaのデータ取得 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ave_s_eta = 0; % s_i/etaの平均値
    for iu = 1:num_UAV
        ave_s_eta=ave_s_eta+X_I{iu}(i,4)/eta{iu};
    end
    ave_s_eta = ave_s_eta/num_UAV;
    
    for iu = 1:num_UAV
        % % % % % 慣性座標における状態の取り出し % % % % %
        xI{iu} = X_I{iu}(i,1);
        yI{iu} = X_I{iu}(i,2);
        psi{iu} = X_I{iu}(i,3);
        s{iu} = X_I{iu}(i,4);
        Delta{iu}(i,1) = X_I{iu}(i,4)/eta{iu} - ave_s_eta; % 作戦３
        
        % 回転行列の計算
        % 機体座標系⇔慣性座標系の変換行列
        SyIB{iu}=[cos(psi{iu}) sin(psi{iu});   %ヨー回転
            -sin(psi{iu}) cos(psi{iu})];
        
        if i > 1.5 % 1ステップ前の値を使用
            Vg_vec = SyIB{iu}*[V{iu}(i-1,1);0]+[Wx;Wy];   % 慣性座標系の速度に風を足す
            chi{iu} = atan2(-Vg_vec(2),Vg_vec(1));   % 慣性座標系
        else
            Vg_vec = SyIB{iu}*[15;0]+[Wx;Wy];   % 慣性座標系の速度に風を足す
            chi{iu} = atan2(-Vg_vec(2),Vg_vec(1));   % 慣性座標系
        end
        Chi_save{iu}(i,1) = chi{iu}; % chiの時間推移を保存
    end
% Step2: セレフレネ座標のデータ取得 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for iu = 1:num_UAV
        F{iu}(i,:) = F_PATH_PPG_r3(Cxi{iu}(i,:))'; % 慣性座標系における目標位置
        kappa{iu} = F{iu}(i,5); % 曲率
        
        % 方位の計算
        x_e_vec{iu}(i,3) = - chi{iu} + F{iu}(i,3); % セレフレネ座標系
        while (x_e_vec{iu}(i,3) < -pi || x_e_vec{iu}(i,3) >= pi) % 0 <= chi_e < 2*pi の否定
            if (x_e_vec{iu}(i,3) > pi)
                x_e_vec{iu}(i,3) = x_e_vec{iu}(i,3) - 2*pi;
            elseif (x_e_vec{iu}(i,3) < -pi)
                x_e_vec{iu}(i,3) = x_e_vec{iu}(i,3) + 2*pi;
            end
        end
        % % 並進位置の計算
        x_eI{iu} = (X_I{iu}(i,1:2) - F{iu}(i,1:2))';   % 慣性座標系における目標位置との差

        % 慣性座標系⇔セレフレネ座標系の変換行列
        SyIF0{iu}=[cos(F{iu}(i,3)) -sin(F{iu}(i,3));   %ヨー回転
            sin(F{iu}(i,3)) cos(F{iu}(i,3))];
        x_e_vec{iu}(i,1:2) = (SyIF0{iu}*x_eI{iu})';
        x_e_vec{iu}(i,4) = Delta{iu}(i,1);
        x_e_vec{iu}(i,5) = s{iu};   % s の代入
        x_e_vec{iu}(i,6) = F{iu}(i,6);   % xi(s)の値を取得
        
        % セレフレネ座標系における位置，姿勢
        x_e{iu} = x_e_vec{iu}(i,1);
        y_e{iu} = x_e_vec{iu}(i,2);
        chi_e{iu} = x_e_vec{iu}(i,3);
    end
% Step3: u_s, u_vの計算 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for iu = 1:num_UAV
        u_s{iu} = - Fg{1}(1,:) * x_e_vec{iu}(i,1:4)';
        u_v{iu}(i,1) = -Fg{1}(3,:)*x_e_vec{iu}(i,1:4)'; % 作戦１
    end
% Step4: 対地速度および対気速度の計算 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    for iu = 1:num_UAV
        Vg{iu}(i,1) = (eta{iu}*(V0 + u_v{iu}(i,1))-u_s{iu})/cos(chi_e{iu});

        % 対気速度の計算
        % キネマティクスの式より逆算する．
        % sin or cos の値が小さくなると誤差が大きくなるので，大きい方で場合分け
        if psi{iu} < 135*pi/180 && psi{iu}>-135*pi/180 && abs(psi{iu})>45*pi/180
            V{iu}(i,1) = (Vg{iu}(i,1)*sin(chi{iu})+Wy)/sin(psi{iu});
        else
            V{iu}(i,1) = (Vg{iu}(i,1)*cos(chi{iu})-Wx)/cos(psi{iu});
        end
        
        % 対気速度範囲を外れた場合の処理
        if V{iu}(i,1) > V_max % 範囲越えの場合
            V{iu}(i,1) = V_max; % 対気速度が最大値でサチるように設定
            % 対地速度と入力量を再計算
            if psi{iu} < 135*pi/180 && psi{iu}>-135*pi/180 && abs(psi{iu})>45*pi/180
                Vg{iu}(i,1)=(V{iu}(i,1)*sin(psi{iu})-Wy)/sin(chi{iu});
            else
                Vg{iu}(i,1)=(V{iu}(i,1)*cos(psi{iu})+Wx)/cos(chi{iu});
            end
            u_s{iu} = eta{iu}*(V0-u_v{iu}(i,1))-Vg{iu}(i,1)*cos(chi_e{iu}); % Vg修正分の誤差をu_sで吸収
        elseif  V{iu}(i,1) < V_min % 範囲越えの場合
             V{iu}(i,1) = V_min; % 対気速度が最小値でサチるように設定
             % 対地速度を再計算
            if psi{iu} < 135*pi/180 && psi{iu}>-135*pi/180 && abs(psi{iu})>45*pi/180
                Vg{iu}(i,1)=(V{iu}(i,1)*sin(psi{iu})-Wy)/sin(chi{iu});
            else
                Vg{iu}(i,1)=(V{iu}(i,1)*cos(psi{iu})+Wx)/cos(chi{iu});
            end
            u_s{iu} = eta{iu}*(V0-u_v{iu}(i,1))-Vg{iu}(i,1)*cos(chi_e{iu}); % Vg修正分の誤差をu_sで吸収
        end
        if Vg{iu}(i,1) > V_max+(Wx^2+Wy^2)^(1/2) % 範囲越えの場合
            Vg{iu}(i,1) = V_max+(Wx^2+Wy^2)^(1/2);
        elseif Vg{iu}(i,1) < V_min-(Wx^2+Wy^2)^(1/2)
            Vg{iu}(i,1) = V_min-(Wx^2+Wy^2)^(1/2);
        end
    end
% Step5: u_chiの計算 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
    for iu = 1:num_UAV
        % メンバシップ関数の計算
        if chi_e{iu} == 0 % chi_eの0割の対処
            M1{iu}(i,1) = 1;
            M2{iu}(i,1) = 0;
        else
            M1{iu}(i,1) = (Vg{iu}(i,1)*sin(chi_e{iu})-V_min*epsilon1*chi_e{iu})/((V_max-V_min*epsilon1)*chi_e{iu});
            M2{iu}(i,1) = (V_max*chi_e{iu}-Vg{iu}(i,1)*sin(chi_e{iu}))/((V_max-V_min*epsilon1)*chi_e{iu});
        end
        N1{iu}(i,1) = ((Vg{iu}(i,1)*cos(chi_e{iu})+u_s{iu})*kappa{iu}-D_kappa_min)/(D_kappa_max-D_kappa_min);
        N2{iu}(i,1) = (D_kappa_max-(Vg{iu}(i,1)*cos(chi_e{iu})+u_s{iu})*kappa{iu})/(D_kappa_max-D_kappa_min);  

        if M1{iu}(i,1) > 1
            M1{iu}(i,1) = 1;
            M2{iu}(i,1) = 0;
        elseif M1{iu}(i,1) < 0
            M1{iu}(i,1) = 1;
            M2{iu}(i,1) = 0;
        end
        if N1{iu}(i,1) > 1
            N1{iu}(i,1) = 1;
            N2{iu}(i,1) = 0;
        elseif N1{iu}(i,1) < 0
            N1{iu}(i,1) = 1;
            N2{iu}(i,1) = 0;
        end
        
        h{iu}(i,1) = M1{iu}(i,1)*N1{iu}(i,1);
        h{iu}(i,2) = M2{iu}(i,1)*N1{iu}(i,1);
        h{iu}(i,3) = M1{iu}(i,1)*N2{iu}(i,1);
        h{iu}(i,4) = M2{iu}(i,1)*N2{iu}(i,1);

        % 制御入力
        gain{iu}=0;
        for ii = 1:length(Fg)
            gain{iu} = gain{iu} + h{iu}(i,ii) * Fg{ii};            
        end
        u_vec{iu}(i,:) = - gain{iu} * x_e_vec{iu}(i,1:4)';

        check(i,1)= u_vec{iu}(i,1)-u_s{iu}; % 線形コントローラとファジィコントローラに誤差がないか確認
        check(i,2)= u_vec{iu}(i,3)-u_v{iu}(i,1); % 線形コントローラとファジィコントローラに誤差がないか確認
        u_vec{iu}(i,1)=u_s{iu}; % 速度がサチるときu_sで誤差を吸収しているため更新
    end
    
% Step6: dot_s, dpsiの計算 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for iu = 1:num_UAV
        dot_s{iu} = u_vec{iu}(i,1) + Vg{iu}(i,1)*cos(chi_e{iu});
        % セレ・フレネ座標系と慣性座標系の間の角速度の変換
        dchi{iu} = -u_vec{iu}(i,2)+F{iu}(i,4);
        dpsi{iu} = Vg{iu}(i,1)*dchi{iu} / (V{iu}(i,1)*cos(chi{iu}-psi{iu}));
%         % FX_79 最大旋回速度 90[deg./s]と仮定
        if dpsi{iu} > 90 * pi / 180
            dpsi{iu} = 90 * pi / 180;
        elseif dpsi{iu} < - 90 * pi / 180
            dpsi{iu} = -90 * pi / 180;
        end

% % %         % 参考文献用　旋回速度の上限値を設定
%         if dpsi{iu} > 0.54
%             dpsi{iu} = 0.54;
%         elseif dpsi{iu} < - 0.54
%             dpsi{iu} = -0.54;
%         end
    end

%Step7: 位置データの更新 (実機の場合はsの情報のみ更新) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ave_ds_eta = 0;   % dot_Deltaの平均値の計算
    for iu = 1:num_UAV
        ave_ds_eta=ave_ds_eta+dot_s{iu}/eta{iu};
    end
    ave_ds_eta = ave_ds_eta/num_UAV;
    
    for iu = 1:num_UAV
        % 慣性座標系における各変数の微分値の計算
        dx{iu} = V{iu}(i,1)*cos(psi{iu})+Wx;
        dy{iu} = -V{iu}(i,1)*sin(psi{iu})+Wy;
        
        dX_I{iu}(i,1:4) = [dx{iu};dy{iu};dpsi{iu};dot_s{iu}];

        % 慣性座標系における状態の更新
        X_I{iu}(i+1,1) = X_I{iu}(i,1) + dx{iu}*dt;
        X_I{iu}(i+1,2) = X_I{iu}(i,2) + dy{iu}*dt;
        X_I{iu}(i+1,3) = X_I{iu}(i,3) + dpsi{iu}*dt;
        while (X_I{iu}(i+1,3) < -pi || X_I{iu}(i+1,3) >= pi) % 0 <= chi_e < 2*pi の否定
            if (X_I{iu}(i+1,3) > pi)
                X_I{iu}(i+1,3) = X_I{iu}(i+1,3) - 2*pi;
            elseif (X_I{iu}(i+1,3) < -pi)
                X_I{iu}(i+1,3) = X_I{iu}(i+1,3) + 2*pi;
            end
        end
        X_I{iu}(i+1,4) = X_I{iu}(i,4) + dot_s{iu}*dt;
        % 経路計算用に変数をまとめたものの更新
        Cxi{iu}(i+1,:) = [s{iu} + dot_s{iu}*dt, dot_s{iu}, t+dt, dt, s{iu}, F{iu}(i,6), F{iu}(i,7), iu, eta{iu}, s0{iu}];
    end

%おまけ: セレフレネ座標系における各状態変数の微分値の計算 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% シミュレーションには不使用
% データの保存のみ
    for iu = 1:num_UAV
        dx_e{iu} = Vg{iu}(i,1)*cos(chi_e{iu}) - dot_s{iu}*(1-kappa{iu}*y_e{iu});
        dy_e{iu} = Vg{iu}(i,1)*sin(chi_e{iu}) - dot_s{iu}*kappa{iu}*x_e{iu};
        dchi_e{iu} = u_vec{iu}(i,2);
        dDelta{iu} = dot_s{iu}/eta{iu} - ave_ds_eta;
        
        dx_e_vec{iu}(i,:) = [dx_e{iu};dy_e{iu};dchi_e{iu};dDelta{iu};dot_s{iu};0];  % 状態変数の速度
    end
    
    for iu = 1:num_UAV % 評価値の計算
        J_sum{iu} = J_sum{iu} + (x_e_vec{iu}(i,1:4)*x_e_vec{iu}(i,1:4)'+u_vec{iu}(i,:)*u_vec{iu}(i,:)')*dt;
%         J_sum{iu} = J_sum{iu} + (x_e_vec{iu}(i,1:4)*x_e_vec{iu}(i,1:4)'+u_vec{iu}(i,:)*[1*10^4 0 0;0 1*10^8 0; 0 0 1*10^3]*u_vec{iu}(i,:)')*dt;
    end
end

% % % % % 無駄なデータを消去 % % % % %
for iu = 1:num_UAV
    X_I{iu}(end,:) = [];
    J_sum{iu} % 評価値の表示
end

%% 計算結果のプロット
% % 状態変数の時間推移
figure('Position',[50 380 570 250])
subplot(2,1,1)
grid on
hold on
for iu = 1:num_UAV
%     plot(time,x_e_vec{iu}(:,5),'color',CV(iu,:))
    plot(time,F{iu}(:,6),'color',CV(iu,:))
end
% ylabel('s[m]')
ylabel('\xi[m]')

subplot(2,1,2)
grid on
hold on
for iu = 1:num_UAV
    plot(time,dX_I{iu}(:,4),'color',CV(iu,:))
end

title('弧長および弧長速度の時間推移')
xlabel('time[sec.]')
ylabel('dot s[m/s]')

%%
figure('Position',[50 230 570 400])
subplot(4,1,1)
grid on
hold on
for iu = 1:num_UAV
    plot(time,x_e_vec{iu}(:,1),'color',CV(iu,:))
end
%title('状態変数の時間推移')
set( gca, 'FontName','Times','FontSize',10 );
ylabel('x_e[m]','FontName','Times','FontSize',12)

subplot(4,1,2)
grid on
hold on
for iu = 1:num_UAV
    plot(time,x_e_vec{iu}(:,2),'color',CV(iu,:))
end
set( gca, 'FontName','Times','FontSize',10 );
ylabel('y_e[m]', 'FontName','Times','FontSize',12 )

subplot(4,1,3)
grid on
hold on
for iu = 1:num_UAV
    plot(time,x_e_vec{iu}(:,3)*180/pi,'color',CV(iu,:))
end
set( gca, 'FontName','Times','FontSize',10 );
ylabel('\chi_e[deg.]', 'FontName','Times','FontSize',12 )

subplot(4,1,4)
grid on
hold on
for iu = 1:num_UAV
    plot(time,Delta{iu}(:,1),'color',CV(iu,:))
end
set( gca, 'FontName','Times','FontSize',10 );
% ylim([-10,10])
xlabel('time[s]', 'FontName','Times','FontSize',12 )
ylabel('\Delta[m]', 'FontName','Times','FontSize',12 )

% % 入力量の時間推移
figure('Position',[650 380 570 250])
subplot(3,1,1)
grid on
hold on
for iu = 1:num_UAV
    plot(time,u_vec{iu}(:,1),'color',CV(iu,:))
end
title('入力量の時間推移')
ylabel('u s')

subplot(3,1,2)
grid on
hold on
for iu = 1:num_UAV
    plot(time,u_vec{iu}(:,2)*180/pi,'color',CV(iu,:))
end
ylabel('u psi')
xlabel('time[sec.]')

subplot(3,1,3)
grid on
hold on
for iu = 1:num_UAV
    plot(time,u_vec{iu}(:,3),'color',CV(iu,:))
end
ylabel('u v')
xlabel('time[sec.]')

% subplot(4,1,4)
% grid on
% hold on
% plot(time,Us{1}(:,1)*180/pi,'b')
% plot(time,UsC(:,1)*180/pi,'r')
% ylabel('u s')
% xlabel('time[sec.]')

% % 慣性座標位置，姿勢の時間推移
figure('Position',[50 50 570 250])
subplot(3,1,1)
grid on
hold on
for iu = 1:num_UAV
    plot(time,X_I{iu}(:,1),'color',CV(iu,:))
end
title('慣性座標位置，姿勢の時間推移')
ylabel('x[m]')

subplot(3,1,2)
grid on
hold on
for iu = 1:num_UAV
    plot(time,X_I{iu}(:,2),'color',CV(iu,:))
end
ylabel('y[m]')

subplot(3,1,3)
grid on
hold on
for iu = 1:num_UAV
    plot(time,X_I{iu}(:,3)*180/pi,'color',CV(iu,:))
    plot(time,Chi_save{iu}(:,1)*180/pi,'m')
end
legend('psi')
xlabel('time[sec.]')
ylabel('psi[deg.]')

% % 真の対地速度と入力速度
figure('Position',[650 50 570 350])
grid on
hold on
for iu = 1:num_UAV
%     plot(time,k_sav{iu}.*V{iu},'color',CV(iu,:))
    plot(time,Vg{iu},'-.','color',CV(iu,:))
    plot(time,V{iu},'color',CV(iu,:))
%     plot(time,Vg{iu},'m')
end
%legend('V ground','V air')
% legend('')
%title('対気速度，対地速度')
set( gca, 'FontName','Times','FontSize',10 );
ylabel('V_g[m/s]', 'FontName','Times','FontSize',12)
xlabel('time[s]', 'FontName','Times','FontSize',12)

% % メンバーシップ関数の時間推移
figure('Position',[1950 -100 750 700])
subplot(2,1,1)
grid on
hold on
for iu = 1:num_UAV
    plot(time,M1{iu},'color',CV(iu,:))
end
title('メンバシップ関数')
ylabel('K1')

subplot(2,1,2)
grid on
hold on
for iu = 1:num_UAV
    plot(time,N1{iu},'color',CV(iu,:))
end
ylabel('M1')
xlabel('time[sec.]')

%% ３次元飛行軌跡
% figure
% set(gca,'YDir','reverse')
% grid on
% hold on
% plot3(F{1}(:,2),F{1}(:,1),ZV,'g','linewidth',2)
% for iu = 1:num_UAV
%     plot3(X_I{iu}(:,2),X_I{iu}(:,1),ZV,'color',CV(iu,:),'linewidth',1)
% end
% axis equal
% title('目標経路と飛行軌跡')
% legend('目標軌道')
% xlabel('y[m]')
% ylabel('x[m]')
% zlabel('z[m]')
% az = 180;
% el = 90;
% view(az, el)
% 
% for iu = 1:num_UAV
%     figure
%     set(gca,'YDir','reverse')
%     grid on
%     hold on
%     plot3(F{iu}(:,2),F{iu}(:,1),ZV,'g','linewidth',2)
%     plot3(X_I{iu}(:,2),X_I{iu}(:,1),ZV,'color',CV(iu,:),'linewidth',1)
%     axis equal
%     title('目標経路と飛行軌跡')
%     legend('目標経路','飛行軌跡')
%     xlabel('y[m]')
%     ylabel('x[m]')
%     zlabel('z[m]')
%     az = 180;
%     el = 90;
%     view(az, el)
% end

%% アニメーション準備
if anime_ON == 1
    SS = SS*0.1;
elseif anime_ON == 2
    SS = SS*0.2;
end

%% アニメーション
end_time = time(end);
time = time';
min_X = 0;
min_Y = 0;
max_X = 0;
max_Y = 0;
id = 0;
for iu = 1:num_UAV
    % 速度調整
    Xanime_n{iu} = [X_I{iu}(:,1:2),ZV,ZV,X_I{iu}(:,3:4),ZV,F{iu}(:,1:2),ZV];

    for n = 1:10
        Xanime{iu}(:,n) = interp1(time,Xanime_n{iu}(:,n),0:SS:end_time);
    end
    t_3(:,1) = interp1(time,time,0:SS:end_time);

    % データ範囲
    if min_X > min([Xanime_n{iu}(:,1);Xanime_n{iu}(:,8)])
        min_X = min([Xanime_n{iu}(:,1);Xanime_n{iu}(:,8)]);
    end
    if min_Y > min([Xanime_n{iu}(:,2);Xanime_n{iu}(:,9)])
        min_Y = min([Xanime_n{iu}(:,2);Xanime_n{iu}(:,9)]);
    end
    if max_X < max([Xanime_n{iu}(:,1);Xanime_n{iu}(:,8)])
        max_X = max([Xanime_n{iu}(:,1);Xanime_n{iu}(:,8)]);
    end
    if max_Y < max([Xanime_n{iu}(:,2);Xanime_n{iu}(:,9)])
        max_Y = max([Xanime_n{iu}(:,2);Xanime_n{iu}(:,9)]);
    end
end
min_Z = -10;
max_Z = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%アニメーション生成
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scrsz = get(groot,'ScreenSize');

for Range=1:fig_num
figure('Position',[scrsz(3)/2+50 100 scrsz(3)/2-100 scrsz(4)-400])

for iu = 1:num_UAV
% for iu = 4:4
    if anime_ON == 0
        Fc{iu}(:,1)=F{iu}((length(time)-1)/fig_num*(Range-1)+1:(length(time)-1)/fig_num*Range,1);
        Fc{iu}(:,2)=F{iu}((length(time)-1)/fig_num*(Range-1)+1:(length(time)-1)/fig_num*Range,2);
        X_Ic{iu}(:,1)=X_I{iu}((length(time)-1)/fig_num*(Range-1)+1:(length(time)-1)/fig_num*Range,1);
        X_Ic{iu}(:,2)=X_I{iu}((length(time)-1)/fig_num*(Range-1)+1:(length(time)-1)/fig_num*Range,2);
        ZVc=ZV((length(time)-1)/fig_num*(Range-1)+1:(length(time)-1)/fig_num*Range,1);
        hold on

        % 目標経路の線のプロット
%         if iu == 5
% %             plot3(Fc{iu}(:,1),Fc{iu}(:,2),ZVc,'g--','linewidth',1)
%             ii = 1;
%             for i = 0:0.001:2*pi
%                 DP_x(ii) = (150*cos(i))/(sin(i)^2 + 1);
%                 DP_y(ii) = (150*cos(i)*sin(i))/(sin(i)^2 + 1);
%                 DP_z(ii) = 0;
%                 ii = ii+1;
%             end
%             plot3(DP_x,DP_y,DP_z,'g--','linewidth',1)
%         end

%         if iu == 4
% %             plot3(Fc{iu}(:,1),Fc{iu}(:,2),ZVc,'g--','linewidth',1)
%             ii = 1;
%             for i = 0:0.001:2*pi
%                 DP_x(ii) = 280*cos(i);
%                 DP_y(ii) = 240*sin(i);
%                 DP_z(ii) = 0;
%                 ii = ii+1;
%             end
%             plot3(DP_x,DP_y,DP_z,'g--','linewidth',1)
%         end

%         plot3(Fc{iu}(:,1),Fc{iu}(:,2),ZVc,'-.','color',CV(iu,:),'linewidth',1)
            ii = 1;
            for i = 0:0.001:2*pi
                DP_x(ii) = 150*eta{iu}*cos(i);
                DP_y(ii) = 100*eta{iu}*sin(i);
                DP_z(ii) = 0;
                ii = ii+1;
            end
            plot3(DP_x,DP_y,DP_z,'g--','color',CV(iu,:),'linewidth',1)
    end
end

for iu = 1:num_UAV
    if anime_ON == 0
        % 飛行軌跡の線のプロット
        plot3(X_Ic{iu}(:,1),X_Ic{iu}(:,2),ZVc,'color',CV(iu,:),'linewidth',1.3)
    end
end

M = moviein(length(Xanime{1}));

%%%
for i=(length(Xanime{1}(:,1))-1)/fig_num*(Range-1)+1:1:(length(Xanime{1}(:,1))-1)/fig_num*Range+1
    for iu = 1:num_UAV
        plot3(Xanime{iu}(i,8),Xanime{iu}(i,9),Xanime{iu}(i,10),'d','color',CV(iu,:),'linewidth',2)
    end
end
%%%

for i=(length(Xanime{1}(:,1))-1)/fig_num*(Range-1)+1:1:(length(Xanime{1}(:,1))-1)/fig_num*Range+1
    for iu = 1:num_UAV
        ppgX=Xanime{iu}(i,1);
        ppgY=Xanime{iu}(i,2);
        ppgZ=Xanime{iu}(i,3);
        p=0;
        q=Xanime{iu}(i,4);
        r=Xanime{iu}(i,5);
        Sroll=[1 0 0;
            0 cos(p) sin(p);
            0 -sin(p) cos(p)];
        Spitch=[cos(q) 0 -sin(q)
            0 1 0 ;
            sin(q) 0 cos(q)];
        Syaw=[cos(r) sin(r) 0;
            -sin(r) cos(r) 0 ;
            0 0 1];
        S=Syaw*Spitch*Sroll;
        PPB=S*[25;0;0];

        if anime_ON >= 1
            plot3(Xanime{iu}(1:i,8),Xanime{iu}(1:i,9),Xanime{iu}(1:i,10),'g','linewidth',1)
            hold on;
            plot3(Xanime{iu}(1:i,1),Xanime{iu}(1:i,2),Xanime{iu}(1:i,3),'color',CV(iu,:))
        end
% 
      % plot3(Xanime{iu}(i,8),Xanime{iu}(i,9),Xanime{iu}(i,10),'go','color',CV(iu,:),'linewidth',4)
        plot3(Xanime{iu}(i,1),Xanime{iu}(i,2),Xanime{iu}(i,3),'o','color',CV(iu,:),'linewidth',5)
        plot3([ppgX PPB(1)+ppgX],[ppgY PPB(2)+ppgY],[ppgZ PPB(3)+ppgZ],'color',CV(iu,:),'linewidth',2)            
        
%         id_m = 30;
%         if i <= id_m
%             plot3(Xanime{iu}(i-id:i,8),Xanime{iu}(i-id:i,9),Xanime{iu}(i-id:i,10),'g','linewidth',1)
%         else
%             plot3(Xanime{iu}(i-id_m:i,8),Xanime{iu}(i-id_m:i,9),Xanime{iu}(i-id_m:i,10),'g','linewidth',1)            
%         end
%         hold on;
%         plot3(Xanime{iu}(i,8),Xanime{iu}(i,9),Xanime{iu}(i,10),'go','linewidth',3)
%         plot3(Xanime{iu}(i,1),Xanime{iu}(i,2),Xanime{iu}(i,3),'o','color',CV(iu,:),'linewidth',5)
%         plot3([ppgX PPB(1)+ppgX],[ppgY PPB(2)+ppgY],[ppgZ PPB(3)+ppgZ],'color',CV(iu,:),'linewidth',2)            

        V_vec(i,1) = PPB(1)+ppgX;
        V_vec(i,2) = PPB(2)+ppgY;
        V_vec(i,3) = PPB(3)+ppgZ;

        set( gca, 'FontName','Times','FontSize',16 );
        xlabel( 'x[m]', 'FontName','Times','FontSize',16 );
        ylabel('y[m]', 'FontName','Times','FontSize',16)
        zlabel('z[m]', 'FontName','Times','FontSize',16)
        grid on;

        view(az_a,el_a)

        % % 動画固定
        axis equal;
        axis([min_X-7*PPG_size,max_X+7*PPG_size,min_Y-7*PPG_size,max_Y+7*PPG_size,min_Z-7*PPG_size,max_Z+7*PPG_size]);
    end
    id = id + 1;

    if anime_ON >= 1
        hold off
        drawnow;
    end
end
% legend('desired path','flight path','FontSize', 10)
% legend('desired path','UAV1','UAV2','UAV3','UAV4','UAV5','FontSize', 10)

end