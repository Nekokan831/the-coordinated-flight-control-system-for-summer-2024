clc
clear
close all
 
% アニメーション設定
anime_ON = 0;   % アニメーションの　オン・オフ 0:なし，1:ひとつ，2:いろんな角度
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
%x_eI_0 = [2.6652,33.816,20*pi/180]; %x_0,y_0,psi_0 %rem
x_eI_0 = [0,0,0]; %x_0,y_0,psi_0 %rem

phi_0 = 0;
p_0 = 0;


%風の設定
% Wx = 2*sin(2*time);
% Wy = 2*sin(2*time);

% Wx = 1;
% Wy = 1;

%UAV速度設定
%V = 12; %対気速度[m/s]
V = 12;

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
    
    %参照経路情報
    F(i,:) = F_PATH_FX79_r1(Cxi(i,:))';
    %↓↓↓
    chi_d = F(i,3);  % 目標航路角[rad]
    
    dchi_d(i,1) = F(i,4); % 目標航路角速度 [rad/s]
  
    kappa = F(i,5);  % 曲率[rad/m]
    xi = F(i,6);     % 媒介変数 xi（ζ）
    
    %慣性座標系におけるx,yの目標との偏差
    x_eI = (X_I(i,1:2) - F(i,1:2))';
    
    %対地速度Vgの計算
     % 機体座標系{B} → 慣性座標系{I}の回転行列
    SyIB=[cos(psi) sin(psi);  % ヨー回転
        -sin(psi) cos(psi)];
    % 対地速度ベクトル
    V_g_vec = SyIB*[V; 0] + [Wx; Wy];  % [m/s]
    
    % 対地速度の大きさ
    V_g(i,1) = norm(V_g_vec);  % [m/s]
    % 航路角の導出
    chi = atan2(-V_g_vec(2),V_g_vec(1));  % [rad]
    
    
    %% 慣性座標系からセレ・フレネ座標系への変換
    x_e_vec(i,3) = -chi + chi_d; %χe
    while (x_e_vec(i,3) < -pi || x_e_vec(i,3) >= pi) % 0 <= chi_e < 2*pi の否定
        if (x_e_vec(i,3) > pi)
            x_e_vec(i,3) = x_e_vec(i,3) - 2*pi;
        elseif (x_e_vec(i,3) < -pi)
            x_e_vec(i,3) = x_e_vec(i,3) + 2*pi;
        end
    end
    
    GammaChi(i,1) = chi;  % 保存用
    
    %慣性座標系→セレ・フレネ座標系の回転行列
    SyIF0=[cos(chi_d) -sin(chi_d);
        sin(chi_d) cos(chi_d)];
    
    x_e_vec(i,1:2) = (SyIF0*x_eI)'; %x,y偏差について慣性座標系からセレ・フレネ座標系xe,yeへ
    
    x_e_vec(i,4) = s;   % s の代入
    x_e_vec(i,5) = xi;   % xi(s)の値を取得
    
    %セレ・フレネ座標系における変数x_e_vecについて分かりやすく表す．
    x_e = x_e_vec(i,1);  % [m]
    y_e = x_e_vec(i,2);  % [m]
    chi_e = x_e_vec(i,3);  % [rad]
    
    
    %% 微分方程式の計算
    
    %定数
    g=9.80665;
    rho=1.155; %ρ
    Ss=0.42;
    ww=2; 
    d0=0.2393;     %δe^0
    Ixx=0.1580;    
    
    Mc=rho*Ss*ww*V^2*(0.1659*d0+0.2578)/2;
    
%     %パラメータ%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     a=0.5;
%     b=0.2; %大で即応性up.0.2良し．大きすぎると発散.0.2以上x
%     c=10;
%     
%     %ゲイン%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     fp = 0.1;
%     fd = 0.1;
%% param      
      a = 40;
      b = 0.8;
      c = 0;
      
      fp = 1.7;
      fd = 0.15;
    
    %% 目標ロール角の計算
    %目標ロール角の計算
    phi_r(i,1) = atan((V_g(i,1)/(a*g))*(b*(y_e+a*chi_e)+V_g(i,1)*sin(chi_e)-V_g(i,1)*kappa*x_e*cos(chi_e)+a*dchi_d(i,1)-c*kappa*x_e^2));
    
    
    %% 目標ロール角にLPF
    Tp = 0.4;
    if i == 1
        phi_r_f(i,1) = phi_r(i,1);
        D_phi_r_f(i,1) = 0;
    else
        D_phi_r_f(i,1) = (1/Tp)*(phi_r(i,1) - phi_r_f(i-1,1));
        phi_r_f(i,1) = D_phi_r_f(i,1)*dt + phi_r_f(i-1,1);
    end
    
    %（LPFなしの目標ロール角速度）
    if i == 1
        dphi_r(i,1) = 0;
    else
        dphi_r(i,1) = (phi_r(i,1)-phi_r(i-1,1))/dt;
    end 
    %% エルロン入力の計算
    
    %delta_a(i,1) = -fp*(phi(i,1)-phi_r_f(i,1))-fd*(p(i,1) - D_phi_r_f(i,1));
    
    %（LPFなしのエルロン入力）
    delta_a(i,1) = -fp*(phi(i,1)-phi_r(i,1))-fd*(p(i,1) - dphi_r(i,1));

    %delta_a_x(i,1) = -fp*(phi(i,1)-phi_r(i,1))-fd*(p(i,1) - dphi_r(i,1));

        
    % エルロン入力の上限下限値を設定
    if delta_a(i,1)>=20*pi/180
        delta_a(i,1)=20*pi/180;
    elseif delta_a(i,1)<=-20*pi/180
        delta_a(i,1)=-20*pi/180;
    end
    
%     % エルロン入力の上限下限値を設定
%     if delta_a_x(i,1)>=20*pi/180
%         delta_a_x(i,1)=20*pi/180;
%     elseif delta_a_x(i,1)<=-20*pi/180
%         delta_a_x(i,1)=-20*pi/180;
%     end
    
    %モデル式
    dot_s = c*x_e +V_g(i,1)*cos(chi_e);
    
%     dx = V_g(i,1)*cos(chi);
%     dy = V_g(i,1)*sin(chi);
%     dchi = -g*tan(phi(i,1))/V_g(i,1);
     
    dx_e = V_g(i,1)*cos(chi_e)-dot_s*(1-kappa*y_e);    %1
    dy_e = V_g(i,1)*sin(chi_e)-dot_s*kappa*x_e;        %2
    dchi_e = -g*tan(phi(i,1))/V_g(i,1) + dchi_d(i,1);       %3
    
    
    dphi = p(i,1);                                     %4
    dp = Mc*delta_a(i,1)/Ixx;                          %5
    
    %%
%      % 墜落判定: 横滑り角によって判定
%     error(i,1) = wrapToPi(abs(chi - psi));  % 横滑り角 [rad]
%     error(i,1) = error(i,1)*180/pi;  % 横滑り角 [deg]
%     if error(i,1) > 88
%         disp('ERROR!! 墜落!!')
%            % 風の設定によっては航路角とヨー角の差が90度を超えてしまう．
%            % 現実的にはありえないため，墜落したものと扱う．
%     end
    
    dx_e_vec(i,:) = [dx_e;dy_e;dchi_e;dot_s;0];  % 状態変数の速度
    
    
    %いるか-------
    % セレ・フレネ座標系と慣性座標系の間の角速度の変換
    dchi = - dchi_e + dchi_d(i,1) ;  % 航路角速度 [rad/s]
    GammaChi(i,2) = dchi;   % 保存用
    dpsi = V_g(i,1)*dchi / (V*cos(chi-psi));  % ヨー角速度 [rad/s] 
    
    % キネマティクス
    dx = V*cos(psi) + Wx;
    dy = -V*sin(psi) + Wy;
    
    dX_I(i,1:4) = [dx;dy;dpsi;dot_s];
    
    % 慣性座標系における位置・姿勢の更新（オイラー法による数値積分）
    X_I(i+1,1) = X_I(i,1) + dx*dt;  % 位置 x [m]の更新
    X_I(i+1,2) = X_I(i,2) + dy*dt;  % 位置 y [m]の更新
    X_I(i+1,3) = X_I(i,3) + dpsi*dt;  % ヨー角 psi [rad]の更新
    while (X_I(i+1,3) < -pi || X_I(i+1,3) >= pi) % 0 <= psi < 2*pi の否定
        if (X_I(i+1,3) > pi)
            X_I(i+1,3) = X_I(i+1,3) - 2*pi;
        elseif (X_I(i+1,3) < -pi)
            X_I(i+1,3) = X_I(i+1,3) + 2*pi;
        end
    end
    
    X_I(i+1,4) = X_I(i,4) + dot_s*dt;  % 経路長 s [m]の更新
    
    phi(i+1,1) = phi(i,1) + dphi*dt;
    
    if phi(i+1,1) > 70*pi/180
        phi(i+1,1) = 70*pi/180;
    elseif phi(i+1,1) < -70*pi/180
        phi(i+1,1) = -70*pi/180;
    end
    
    p(i+1,1) =p(i,1) + dp*dt;
    
    if p(i+1,1) > 100*pi/180
        p(i+1,1) = 100*pi/180;
    elseif p(i+1,1) < -100*pi/180
        p(i+1,1) = -100*pi/180;
    end
    
    
    
    Cxi(i+1,:) = [s + dot_s*dt, dot_s, t, dt, s, F(i,6), F(i,7)];   % xi 計算用の変数保存
    
    
end
%% plot

% % % % % 無駄なデータを消去 % % % % %
X_I(end,:) = [];
phi(end,:) = [];
p(end,:) = [];

% 図を保存するかどうか -------------
SAVE = 0;

%シミュレーション番号
ex_num='yotuba';
% 図の保存先フォルダ ---------------------------
%path_save = "C:\Users\";
% フォントサイズ -------------------------------
FSize = 15;  % 候補：10, 15



%% 飛行軌跡（x-y平面）
figure
set(gca,'XDir','reverse')
set(gca,'YDir','reverse')
grid on
hold on

p1 = plot(F(:,1),F(:,2),'g-','linewidth',4);  % 目標経路
p2 = plot(X_I(1,1),X_I(1,2),'mo','linewidth',6);  % 初期位置
p3 = plot(X_I(end,1),X_I(end,2),'bo','linewidth',4);  % 最終位置
p4 = plot(X_I(:,1),X_I(:,2),'b','linewidth',2);  % 飛行経路
axis equal
%('目標経路と飛行経路')
legend([p2, p4, p1],'初期位置','飛行経路','目標経路','Location','best','Fontsize',12)
xlabel('x[m]')
ylabel('y[m]')
ax = gca;
ax.FontSize = 15;

az = 180;
el = 90;
view(az, el)
hold off

if SAVE == 1
    saveas(gcf,fullfile(path_save,['kiseki_s',ex_num]) ,'epsc')
end

%% 状態変数の時間変化
figure;
subplot(3,1,1)
plot(time,x_e_vec(:,1))
grid on
hold on
%title('状態変数の時間変化')
ylabel('x_e [m]')
ax = gca;
ax.FontSize = 12;

subplot(3,1,2)
plot(time,x_e_vec(:,2))
grid on
hold on
ylabel('y_e [m]')
ax = gca;
ax.FontSize = 12;

subplot(3,1,3)
plot(time,x_e_vec(:,3)*180/pi)
grid on
hold on
ylabel('\chi_e [deg]')
xlabel('time [s]')
hold off
ax = gca;
ax.FontSize = 12;

if SAVE == 1
    saveas(gcf,fullfile(path_save,['F_s',ex_num]) ,'epsc')
end

%% e
e=x_e_vec(:,2)+a*x_e_vec(:,3);

figure;
plot(time,e)
grid on
hold on
%title('e')
xlabel('time [s]')
ylabel('e')
ax = gca;
ax.FontSize = 15;

if SAVE == 1
    saveas(gcf,fullfile(path_save,['e_s',ex_num]) ,'epsc')
end


%% エルロン入力

figure;
plot(time,delta_a*180/pi)
grid on
hold on
%plot(time,delta_a_x*180/pi)
%title('エルロン入力')
xlabel('time [s]')
ylabel('\delta_a')
%legend('$\delta_a$','Interpreter','latex')
%legend('$\delta_a$(LPF)','$\delta_a$','Interpreter','latex')
%ylim([-20 20]);
ax = gca;
ax.FontSize = 12;

if SAVE == 1
    saveas(gcf,fullfile(path_save,['aile_s',ex_num]) ,'epsc')
end


%% 慣性座標位置，姿勢
figure;
subplot(3,1,1)
plot(time,X_I(:,1))
grid on
hold on
%title('慣性座標位置，姿勢の時間変化')
xlabel('time [s]')
ylabel('x [m]')
ax = gca;
ax.FontSize = 12;

subplot(3,1,2)
plot(time,X_I(:,2))
grid on
hold on
ylabel('y [m]')
ax = gca;
ax.FontSize = 12;

subplot(3,1,3)
plot(time,unwrap(X_I(:,3))*180/pi)  % ヨー角 [deg]
grid on
hold on
plot(time,unwrap(GammaChi(:,1)*180/pi),'r')  % 航路角 [deg]
legend('\psi','\chi', 'Location', 'best','Fontsize',12)
xlabel('time [s]')
ylabel('\psi [deg]')

if SAVE == 1
    saveas(gcf,fullfile(path_save,['I_s',ex_num]) ,'epsc')
end

%% ロール角と目標ロール角
figure;
plot(time,phi(:,1)*180/pi)
grid on
hold on
plot(time,phi_r(:,1)*180/pi)
%plot(time,phi_r_f(:,1)*180/pi)
%title('ロール角と目標ロール角')
xlabel('time [s]')
ylabel('$\phi$ , $\phi_r$[degree]','Interpreter','latex')
legend('$\phi$','$\phi_r$','Interpreter','latex','FontSize',20)
ax = gca;
ax.FontSize = 12;


if SAVE == 1
    saveas(gcf,fullfile(path_save,['roll_s',ex_num]) ,'epsc')
end

%% ロール角速度と目標ロール角速度
figure;
plot(time,p(:,1)*180/pi)
grid on
hold on
plot(time,dphi_r*180/pi)
%plot(time,D_phi_r_f*180/pi)

%title('ロール角速度と目標ロール角速度')
xlabel('time [s]')
ylabel('$\dot{\phi}$, $\dot{\phi_r}$[degree]','Interpreter','latex')
legend('$\dot{\phi}$', '$\dot{\phi_r}$','Interpreter','latex','FontSize',20)
ax = gca;
ax.FontSize = 12;


if SAVE == 1
    saveas(gcf,fullfile(path_save,['droll_s',ex_num]) ,'epsc')
end


%% 対地速度
figure;
grid on
hold on
plot(time,V_g)
%title('対地速度')
ylabel('V_g [m/s]')
xlabel('time [s]')
ax = gca;
ax.FontSize = 15;

if SAVE == 1
    saveas(gcf,fullfile(path_save,['vg_s',ex_num]) ,'epsc')
end

%% 目標航路角速度

figure;
subplot(2,1,1)
plot(time,F(:,3)*180/pi)
grid on
hold on
ylabel('$$\chi_d$$','Interpreter','latex')
ax = gca;
ax.FontSize = 12;

subplot(2,1,2)
plot(time,F(:,4)*180/pi)
grid on
hold on
%plot(time,dchi_d_f(:,1))
ylabel('$$\dot{\chi_d}$$','Interpreter','latex')
ax = gca;
ax.FontSize = 12;


if SAVE == 1
    saveas(gcf,fullfile(path_save,['chi_d_s',ex_num]) ,'epsc')
end

%% κ,ξ,s

figure;
subplot(3,1,1)
plot(time,F(:,5))
grid on
hold on
ylabel('$$\kappa$$','Interpreter','latex')
ax = gca;
ax.FontSize = 12;

subplot(3,1,2)
plot(time,F(:,6))
grid on
hold on
ylabel('$$\xi$$','Interpreter','latex')
ax = gca;
ax.FontSize = 12;

subplot(3,1,3)
plot(time,X_I(:,4))
grid on
hold on
ylabel('$$s$$','Interpreter','latex')
ax = gca;
ax.FontSize = 12;

if SAVE == 1
    saveas(gcf,fullfile(path_save,['kappa_zeta_s_s',ex_num]) ,'epsc')
end

%% dot_s
figure;
plot(time,dx_e_vec(:,4))
grid on
hold on
ylabel('$$\dot{s}$$','Interpreter','latex')
ax = gca;
ax.FontSize = 12;

if SAVE == 1
    saveas(gcf,fullfile(path_save,['ds_s',ex_num]) ,'epsc')
end

%% アニメーション準備
if anime_ON == 1
    SS = SS*0.1;
elseif anime_ON == 2
    SS = SS*0.2;
end

%% アニメーション
end_time = time(end);
time = time';

% 速度調整
Xanime_n = [X_I(:,1:2),ZV,ZV,X_I(:,3:4),ZV,F(:,1:2),ZV];

for n = 1:10
    Xanime(:,n) = interp1(time,Xanime_n(:,n),0:SS:end_time);
end
t_3(:,1) = interp1(time,time,0:SS:end_time);
% データ範囲
min_X = min([Xanime_n(:,1);Xanime_n(:,8)]);
min_Y = min([Xanime_n(:,2);Xanime_n(:,9)]);
min_Z = -10;
max_X = max([Xanime_n(:,1);Xanime_n(:,8)]);
max_Y = max([Xanime_n(:,2);Xanime_n(:,9)]);
max_Z = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%アニメーション生成
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scrsz = get(groot,'ScreenSize');
figure('Position',[scrsz(3)/2+50 100 scrsz(3)/2-100 scrsz(4)-200])

if anime_ON == 0
    plot3(F(:,1),F(:,2),ZV,'g','linewidth',2)
    hold on
    plot3(X_I(:,1),X_I(:,2),ZV,'b')
end

M = moviein(length(Xanime));
for i=1:1:length(Xanime)
    
    if anime_ON == 2
        subplot(2,2,1)
    end
    ppgX=Xanime(i,1);
    ppgY=Xanime(i,2);
    ppgZ=Xanime(i,3);
    p=0;
    q=Xanime(i,4);
    r=Xanime(i,5);
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
    PPB=S*[15;0;0];
    
    if anime_ON >= 1
        plot3(Xanime(1:i,8),Xanime(1:i,9),Xanime(1:i,10),'g','linewidth',2)
        %     moviemaker2;   % PPG をプロットする場合
        hold on;
        plot3(Xanime(1:i,1),Xanime(1:i,2),Xanime(1:i,3),'b')
    end
    
    plot3(Xanime(i,8),Xanime(i,9),Xanime(i,10),'go','linewidth',8)
    plot3(Xanime(i,1),Xanime(i,2),Xanime(i,3),'ro','linewidth',5)
    
    plot3([ppgX PPB(1)+ppgX],[ppgY PPB(2)+ppgY],[ppgZ PPB(3)+ppgZ],'r','linewidth',2)
    V_vec(i,1) = PPB(1)+ppgX;
    V_vec(i,2) = PPB(2)+ppgY;
    V_vec(i,3) = PPB(3)+ppgZ;
    
    %     title('飛行経路')
    %         xlabel('X[m]')
    set( gca, 'FontName','Times','FontSize',16 );
    xlabel( 'x[m]', 'FontName','Times','FontSize',16 );
    ylabel('y[m]', 'FontName','Times','FontSize',16)
    zlabel('z[m]', 'FontName','Times','FontSize',16)
    grid on;
    
    view(az_a,el_a)
    
    % % 動画固定
    axis equal;
    axis([min_X-7*PPG_size,max_X+7*PPG_size,min_Y-7*PPG_size,max_Y+7*PPG_size,min_Z-7*PPG_size,max_Z+7*PPG_size]);
    
    if anime_ON >= 1
        if anime_ON == 2
            zoom(1.5)
        end
        hold off
        drawnow;
    end
    
    if anime_ON == 2
        subplot(2,2,2)
        plot3(Xanime(1:i,8),Xanime(1:i,9),Xanime(1:i,10),'g','linewidth',2)
        hold on
        plot3(Xanime(1:i,1),Xanime(1:i,2),Xanime(1:i,3),'b')
        plot3(Xanime(i,8),Xanime(i,9),Xanime(i,10),'go','linewidth',8)
        plot3(Xanime(i,1),Xanime(i,2),Xanime(i,3),'ro','linewidth',5)
        plot3([ppgX PPB(1)+ppgX],[ppgY PPB(2)+ppgY],[ppgZ PPB(3)+ppgZ],'r','linewidth',2)
        
        xlabel('X[m]')
        ylabel('Y[m]')
        zlabel('Z[m]')
        grid on;
        
        view(0,0)
        % % 動画固定
        axis equal;
        axis([min_X-7*PPG_size,max_X+7*PPG_size,min_Y-7*PPG_size,max_Y+7*PPG_size,min_Z-7*PPG_size,max_Z+7*PPG_size]);
        zoom(0.8)
        hold off
        drawnow;
        
        subplot(2,2,3)
        plot3(Xanime(1:i,8),Xanime(1:i,9),Xanime(1:i,10),'g','linewidth',2)
        hold on
        plot3(Xanime(1:i,1),Xanime(1:i,2),Xanime(1:i,3),'b')
        plot3(Xanime(i,8),Xanime(i,9),Xanime(i,10),'go','linewidth',8)
        plot3(Xanime(i,1),Xanime(i,2),Xanime(i,3),'ro','linewidth',5)
        plot3([ppgX PPB(1)+ppgX],[ppgY PPB(2)+ppgY],[ppgZ PPB(3)+ppgZ],'r','linewidth',2)
        
        xlabel('X[m]')
        ylabel('Y[m]')
        zlabel('Z[m]')
        grid on;
        
        view(-90,0)
        % % 動画固定
        axis equal;
        axis([min_X-7*PPG_size,max_X+7*PPG_size,min_Y-7*PPG_size,max_Y+7*PPG_size,min_Z-7*PPG_size,max_Z+7*PPG_size]);
        zoom(0.8)
        hold off
        drawnow;
        
        subplot(2,2,4)
        plot3(Xanime(1:i,8),Xanime(1:i,9),Xanime(1:i,10),'g','linewidth',2)
        hold on
        plot3(Xanime(1:i,1),Xanime(1:i,2),Xanime(1:i,3),'b')
        plot3(Xanime(i,8),Xanime(i,9),Xanime(i,10),'go','linewidth',8)
        plot3(Xanime(i,1),Xanime(i,2),Xanime(i,3),'ro','linewidth',5)
        plot3([ppgX PPB(1)+ppgX],[ppgY PPB(2)+ppgY],[ppgZ PPB(3)+ppgZ],'r','linewidth',2)
        
        xlabel('X[m]')
        ylabel('Y[m]')
        zlabel('Z[m]')
        grid on;
        
        view(0,90)
        % % 動画固定
        axis equal;
        axis([min_X-7*PPG_size,max_X+7*PPG_size,min_Y-7*PPG_size,max_Y+7*PPG_size,min_Z-7*PPG_size,max_Z+7*PPG_size]);
        zoom(0.8)
        hold off
        drawnow;
    end
end

%% ２次元平面プロット

if plot2D == 1
    % xyプロット
    figure('Position',[scrsz(3)/2+50 100 scrsz(3)/2-100 scrsz(4)-200])
    plot(F(:,1),F(:,2),'g','linewidth',2)
    hold on
    plot(Xanime(:,8),Xanime(:,9),'go','linewidth',8)
    plot(X_I(:,1),X_I(:,2),'b')
    plot(Xanime(:,1),Xanime(:,2),'ro','linewidth',5)

    for i = 1:length(V_vec)
        plot([Xanime(i,1) V_vec(i,1)],[Xanime(i,2) V_vec(i,2)],'r','linewidth',2)
    end
    axis equal
    axis([min_X-7*PPG_size,max_X+7*PPG_size,min_Y-7*PPG_size,max_Y+7*PPG_size]);

    set( gca, 'FontName','Times','FontSize',16 );
    xlabel( 'x[m]', 'FontName','Times','FontSize',16 );
    ylabel('y[m]', 'FontName','Times','FontSize',16)
    %zlabel('z[m]', 'FontName','Times','FontSize',16)
    grid on;

    % xzプロット
    figure('Position',[scrsz(3)/2+50 100 scrsz(3)/2-100 scrsz(4)-200])
    plot(F(:,1),F(:,3),'g','linewidth',2)
    hold on
    plot(Xanime(:,8),Xanime(:,10),'go','linewidth',8)
    plot(X_I(:,1),X_I(:,3),'b')
    plot(Xanime(:,1),Xanime(:,3),'ro','linewidth',5)

    for i = 1:length(V_vec)
        plot([Xanime(i,1) V_vec(i,1)],[Xanime(i,3) V_vec(i,3)],'r','linewidth',2)
    end
    axis equal
    axis([min_X-7*PPG_size,max_X+7*PPG_size,min_Z-7*PPG_size,max_Z+7*PPG_size]);

    set( gca, 'FontName','Times','FontSize',16 );
    xlabel( 'x[m]', 'FontName','Times','FontSize',16 );
    %ylabel('y[m]', 'FontName','Times','FontSize',16)
    ylabel('z[m]', 'FontName','Times','FontSize',16)
    grid on;
end