% 2021/5/17
% kappa, xi の計算，目標経路の出力プログラム
% 
%clear 
close all
%clc

%% プログラムの使い方，説明
%「目標経路の設定」項目に目標経路を媒介変数xiを用いて表記する．
% 設定した目標経路に対応した Fxi および xi，dPdxi を出力する．

% Fxiの中身，および順番は次の通り．
% 1列目 [ 目標慣性 x 座標，
% 2列目   目標慣性 y 座標, 
% 3列目   目標航路角 chi,
% 4列目   目標航路角速度,
% 5列目   曲率 kappa,
% 6列目   xi,
% 7列目   現時点での i の実数値]

%% シンボリック関数の定義
syms x y          real   % 目標軌道の各要素
syms s dot_s        positive  % 弧長長さ
syms xi dot_xi      positive
syms xi_x xi_y      positive
syms i_sav i dxi

%% UAVの情報（経路に追従可能かどうか判断するのに使用）
% PPGのスペック
dpsi_max = pi/3;  % 最大旋回速度[rad./s]
v_air = 12;  % 速度[m/s]

%% 目標経路の設定
% % % 直線経路----------
% x = xi;
% y = 0;

% WP2点(P0,P1)で定義される直線経路

% syms x0 y0 x1 y1 real
% P0 = [x0 y0];
% P1 = [x1 y1];
% x = (1 - xi)*P0(1) + xi*P1(1);
% y = (1 - xi)*P0(2) + xi*P1(2);



% WP1点（中心点）と半径50で定義される円経路（右旋回）：初期位相 +PI
%syms x0 y0 real
%syms r positive
% x = 50*cos(-xi + pi) + x0;
% y = 50*sin(-xi + pi) + y0;

% x = 50*cos(-xi + 3*pi/4); 
% y = 50*sin(-xi + 3*pi/4); 

%{
% WP1点（中心点）と半径50で定義される円経路（左旋回）：初期位相 +PI
syms x0 y0 real
syms r positive
x = 50*cos(xi + pi) + x0;
y = 50*sin(xi + pi) + y0;
%}

% L01-01
%x = xi;  % 経度方向
%y = X_I(end,2)/X_I(end,1)*xi;  % 緯度方向

% L01-02, L01-03
%x = -xi;  % 経度方向
%y = X_I(end,2)/X_I(end,1)*(-xi);  % 緯度方向

% % % 円経路
% (左旋回)
%x = 50*cos(xi + pi*3/4);  % L02-01
%y = 50*sin(xi + pi*3/4);

%x = 50*cos(xi + pi);  % L02-02
%y = 50*sin(xi + pi);

%x = 50*cos(xi);  % L02-02
%y = 50*sin(xi);

% WP2点で定義される円経路(左旋回/右旋回)
%{
syms x0 y0 x1 y1 real
syms r positive
x = r*cos(-xi) + (x0 + x1)/2;
y = r*sin(-xi) + (y0 + y1)/2;
%}

% (右旋回)
% x = 50*cos(-xi - pi/2);  % L02-03
% y = 50*sin(-xi - pi/2);

% % % 楕円経路
%x = 60*cos(xi);
%y = 30*sin(xi);

% % % sin波の経路
% x = 20*xi;
% y = 50*sin(xi);

% % % sin波の経路
%  x = 40*xi;
%  y = 50*sin(xi);

%%%sin波
% syms a positive
% syms b positive
%  x = a*xi;
%  y = b*sin(xi);


% % % % リサージュ曲線経路，8の字（a=1, b=2）
% syms r positive
%  r = 100;
%  x = 2*r*sin(xi);
%  y = 0.5*r*sin(2*xi);

% 
% x = 100*sin(xi);
% y = 50*sin(2*xi);

% x = 10*sin(xi);
% y = 5*sin(2*xi);

%r=60の8の字 
%   x = 120*sin(xi);
%   y = 60*sin(2*xi);

% x = 50*sin(xi);
% y = 25*sin(2*xi);


%r=70の8の字 
%  x = 140*sin(xi);
%  y = 70*sin(2*xi);

% % % % リサージュ曲線経路（a=3, b=4）
%  x = 220*sin(3*xi);
%  y = 220*sin(4*xi);

% % % % リサージュ曲線経路（a=3, b=4）
% x = 220*sin(3*xi);
% y = 220*sin(4*xi);

% % % インボリュート曲線の経路
% x = 30 * (cos(xi) + xi*sin(xi));
% y = 30 * (sin(xi) - xi*cos(xi));


% % % インボリュート曲線の経路
%  x = 20 * (cos(xi) + xi*sin(xi));
%  y = 20 * (sin(xi) - xi*cos(xi));


% % % 正葉曲線の経路
syms r positive
 x =140 * sin(2*xi) * cos(xi);
 y = 140 * sin(2*xi) * sin(xi);
 
 
% % % 正葉曲線の経路pi/2
% syms r positive
%  x =r * sin(2*(xi-pi/2)) * cos(xi-pi/2);
%  y = r * sin(2*(xi-pi/2)) * sin(xi-pi/2);
 

% % % % % リサージュ曲線経路，電通大（a=5, b=6）
%syms r positive
% r = 200;
% x = r*cos(5*xi);
% y = r*cos(6*xi);

%x = -r*cos(5*xi);
%y = r*cos(6*xi);
%x = -50*cos(5*xi);
%y = 50*cos(6*xi);

% % % レムニスケート経路

% r=20;
% x = 2*r*cos(xi)/(1+(sin(xi))^2);
% y = 2*r*sin(xi)*cos(xi)/(1+(sin(xi))^2);


% r=60;
% x = 2*r*cos(xi-pi/2)/(1+(sin(xi-pi/2))^2);
% y = -2*r*sin(xi-pi/2)*cos(xi-pi/2)/(1+(sin(xi-pi/2))^2);

% syms r positive
% x = r*sin(xi)/(1+(cos(xi))^2);
% y = r*cos(xi)*sin(xi)/(1+(cos(xi))^2);

% x = 100*cos(xi)/(1+(sin(xi))^2);
% y = 100*sin(xi)*cos(xi)/(1+(sin(xi))^2);

% % % アルキメデスの螺旋の経路
% x = 30 * xi * sin(xi);
% y = 30 * xi * cos(xi);

%% 以下，自動計算

%% kappa s をそれぞれ計算
Fxi = [x,y,0];
dFxi = simplify(diff(Fxi,xi))
ddFxi = diff(dFxi,xi)
dddFxi = diff(ddFxi,xi)

kappa = simplify(norm2(cross(dFxi,ddFxi))/norm2(dFxi)^3)

%% 目標軌道を s で表す．
x_xi = Fxi(1);
y_xi = Fxi(2);

%% 目標方位を計算
psi_xi = atan2(-dFxi(2),dFxi(1));
dpsi_xi = simplify(diff(psi_xi,xi)*dot_xi);

%% 目標軌道　計算結果
Fxi = [x_xi,y_xi,psi_xi,dpsi_xi,kappa,xi,i_sav] % 目標経路の出力

dPdxi = simplify(sqrt(dFxi(1,1)^2+dFxi(1,2)^2))
dPdxi = simplify(subs(dPdxi,xi,i*dxi))

s_solve = simplify(int(norm2(dFxi),xi))    
xi = simplify(solve(s_solve - s,xi))

%% 追従可能かどうかの判定
% 曲率 kappa の最大値を計算
PATH_kappa_max = 0;
for xi = 10^(-5):1*pi/180:2*pi
    %xi*180/pi
    kappa_c = double(subs(kappa));
    if PATH_kappa_max < kappa_c
        PATH_kappa_max = kappa_c;
    end
end

% 追従可能な最大曲率
uav_kappa_max = dpsi_max/v_air;

if uav_kappa_max > PATH_kappa_max
    disp('この経路は 追従可能 です!!')
else
    disp('この経路は 追従不可 です・・・')
end
fprintf('追従可能な最大曲率 %d [rad/m] ，\n',uav_kappa_max)
fprintf('この経路の最大曲率は %d [rad/m] です．\n',PATH_kappa_max)

%% 軌跡のプレビュー
close all
xi_plot = [0:0.02:4*pi];

figure;
axis equal
hold on
x_plot = double(subs(x, xi_plot));
y_plot = double(subs(y, xi_plot));
plot(x_plot, y_plot)
plot(x_plot(1), y_plot(1), 'mo')  % xi = 0
plot(double(subs(x, pi/4)), double(subs(y, pi/4)), 'bo')  % xi = pi/4
plot(double(subs(x, pi/2)), double(subs(y, pi/2)), 'bo')  % xi = pi/2
%plot(double(subs(x, pi*3/4)), double(subs(y, pi*3/4)), 'bo')  % xi = pi*3/4
hold off

figure
plot(x_plot)

figure
plot(y_plot)