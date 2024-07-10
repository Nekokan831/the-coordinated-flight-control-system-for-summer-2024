% 2022/3/30 by Yutoku Takahashi
% 2次元モデル　協調制御
% 4ルール

% 論文投稿バージョン
% データ整理

% 最適制御の追加

%% 初期化
close all
clear
clc

tic % 計算時間の計測開始
%% モデルパラメータの設定
Input_constraints = 1; % 入力制限 1:あり, 0:なし

% Vg_min = 5;     % 対地速度 最小値[m/s]
% Vg_max = 25;    % 対地速度 最大値[m/s]

Vg_min = 20;     % 機体速度 最小値[m/s]
Vg_max = 50;    % 機体速度 最大値[m/s]
V0 = 35;   % 対気速度

epsilon1 = sin(178*pi/180) / (178*pi/180);   % sin chi の最小傾き

% kappa_max = 0.05;
% U_x_max = 15;   % 入力制限
% mu_s = 15;
% mu_chi = 90*pi/180;
% mu_v = 10;

% kappa_max = 0.05;
% U_x_max = 45;   % 入力制限
% mu_s = 3000;
% mu_chi = 18000*pi/180;
% mu_v = 2000;

kappa_max = 0.05;
U_x_max = 100;   % 入力制限
mu_s = 100;
mu_chi = 180*pi/180;
mu_v = 30;

D_kappa_max = (Vg_max+U_x_max)*kappa_max;    % 機体速度 最大値[m/s]
D_kappa_min = -D_kappa_max;

x0=[10;10;pi];   % 初期値
% x0=[10;10;90*pi/180];   % 初期値
x0_c=30;   % 初期値
% x0=[100;100;900*pi/180];   % 初期値
% x0_c=100;   % 初期値
% x0=[0;0;0];   % 初期値
% x0_c=0;   % 初期値

% % % % 最小化するlambdaについて
distance_lambda = 100;
lambda_low= -10;
% lambda_low = (1e6);
lambda_up = (1e8);
% lambda_up = (1e14);
% lambda = 1;

% W = eye(3); 
% W_c = 1;
% R = [1*10^4 0
%     0 1*10^8];
% R_c = 1*10^3;

% W = [10^(-4) 0 0
%     0 10^(-18) 0
%     0 0 10^(-18)]; 
% W_c = 1*10^(-3);
% R = eye(2);
% R_c = 1;

W = [10^(-0) 0 0
    0 10^(-0) 0
    0 0 10^(-0)]; 
W_c = 1*10^(-0);
R = [10^3 0
    0 10^4];
R_c = 10^(4);

% W = [10^(-0) 0 0
%     0 10^(-0) 0
%     0 0 10^(-0)]; 
% W_c = 1*10^(-0);
% R = [10^0 0
%     0 10^0];
% R_c = 10^(0);

%% 行列定義
% LMIでFに特殊な制限が必要な関係上，ここでは単体のモデルの未定義
% LMI条件で拡大系を記述

% A{1} = [0 D_kappa_max 0;
%     -D_kappa_max 0 Vg_max;
%     0 0 0];
% 
% A{2} = [0 D_kappa_max 0;
%     -D_kappa_max 0 Vg_min*epsilon1;
%     0 0 0];
% 
% A{3} = [0 D_kappa_min 0;
%     -D_kappa_min 0 Vg_max;
%     0 0 0];
% 
% A{4} = [0 D_kappa_min 0;
%     -D_kappa_min 0 Vg_min*epsilon1;
%     0 0 0];

% 線形化モデル
A{1} = [0 0 0;
    0 0 V0;
    0 0 0];
A{2} = A{1};
A{3} = A{1};
A{4} = A{1};

B{1} = [-1 0;
    0 0;
    0 1];
B{2} = B{1};
B{3} = B{1};
B{4} = B{1};

m_num = length(A);   % メンバシップ関数の数
%% 協調制御用　拡大系の記述
[sA_x,sA_y] = size(A{1}); % A行列のサイズ取得
[sB_x,sB_y] = size(B{1}); % B行列のサイズ取得

A_C{1} = [A{1} zeros([sA_x,1]);
    zeros([1,sA_y]) 0];
A_C{2} = [A{2} zeros([sA_x,1]);
    zeros([1,sA_y]) 0];
A_C{3} = [A{3} zeros([sA_x,1]);
    zeros([1,sA_y]) 0];
A_C{4} = [A{4} zeros([sA_x,1]);
    zeros([1,sA_y]) 0];
B_C{1} = [B{1} zeros([sB_x,1]);
    zeros([1,sB_y]) 1];
B_C{2} = [B{2} zeros([sB_x,1]);
    zeros([1,sB_y]) 1];
B_C{3} = [B{3} zeros([sB_x,1]);
    zeros([1,sB_y]) 1];
B_C{4} = [B{4} zeros([sB_x,1]);
    zeros([1,sB_y]) 1];

x0_C = [x0;x0_c];

%% 可制御性の確認
Mc = [B_C{1} A_C{1}*B_C{1} A_C{1}^2*B_C{1} A_C{1}^3*B_C{1} A_C{1}^4*B_C{1}];
RANK = rank(Mc);

if RANK == length(A_C{1})
    disp('可制御!!')
else
    disp('非可制御．．．一時停止します．．．')
    pause(1000)
end

%% LMIの設定パラメータ

%% LMIの計算　(以下は変更不要)
flag_feas = 0;   % 可解なら 1 になる．

count_for_optimization = 0;
% setlmis([]);   % LMIの初期化
% LMI_calc_gain_2D_PPG_r6;
while (lambda_up-lambda_low)>distance_lambda
% % %     回数を計測
    count_for_optimization = count_for_optimization + 1;
% % %     2分法
    lambda=(lambda_up+lambda_low)/2
    
    setlmis([]);   % LMIの初期化
    LMI_calc_gain_2D_PPG_r6;
end
%% 得られたlambda で再計算
lambda = lambda_up;
setlmis([]);   % LMIの初期化
LMI_calc_gain_2D_PPG_r6;

%% フィードバックゲインの出力

if flag_feas == 1 % 解が得られた場合
    % フィードバックゲインの出力
    
    for i = 1:m_num
%        disp(['F',num2str(i)])
        F{i}=[F11{i} F12{i};0,0,0,F22{i}];
    end
    
    % 差分 F - [F_x,F_chi] を計算
    for i = 1:m_num
        F_ch{i} = F{i}-[Fs11{i},Fs12{i};Fchi11{i},Fchi12{i};0,0,0,F22{i}];
    end
    for ii = 1:sB_y+1
        F_gosa(ii) = 0;
        for i = 1:m_num
            F_gosa(ii) = F_gosa(ii) + F_ch{i}(ii,:)*F_ch{i}(ii,:)';
        end
    end
    
    % 誤差があった場合に出力
    if F_gosa(1) ~= 0 || F_gosa(2) ~= 0 || F_gosa(3) ~= 0
        disp('ゲインが正しく計算できてない，誤差あり')
        F_gosa(1)
        F_gosa(2)
        F_gosa(3)
    else
        disp('再計算OK!!')
    end
    
end
M_C{i}
    for i = 1:m_num
       disp(['F',num2str(i)])
        F{i}
    end

lambda

toc % 計算時間の計測終了





