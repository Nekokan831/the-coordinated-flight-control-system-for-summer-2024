%% 文字設定と初期化
% 初期化
clc
clear
close all

% フォントと文字サイズの設定
set(0,'defaultTextFontSize',15)   %初期値は 9

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
%{UAVの台数目}
x_eI_0{1} = [0,0,0*pi/180]; %x_0,y_0,psi_0 %rem
x_eI_0{2} = [0,0,0*pi/180]; %x_0,y_0,psi_0 %rem


%UAV速度設定
V_a = 12;  %対気速度[m/s]

%初期状態の計算
Cxi0{1} = [0, 0, 0, dt, 0, 0, 0]; % F_PATH へ与える入力の初期状態:s,dot_s,t,dt,s_old,xi_old,i_sar
Cxi0{2} = [0, 0, 0, dt, 0, 0, 0]; % F_PATH へ与える入力の初期状態:s,dot_s,t,dt,s_old,xi_old,i_sar

X_I_0{1} = F_PATH_FX79_r1(Cxi0{1})'+[x_eI_0{1}, 0, 0, 0, 0]'; % 慣性座標系における初期状態：x_d,y_d,χ_d,dχ_d,κ,xi,i
X_I_0{2} = F_PATH_FX79_r1(Cxi0{2})'+[x_eI_0{2}, 0, 0, 0, 0]'; % 慣性座標系における初期状態：x_d,y_d,χ_d,dχ_d,κ,xi,i


%事前割り当て
for iu = 1:2
    x_e_vec{iu} = zeros([length(time),5]); % 誤差ダイナミクスの状態量x_eベクトル，および s, xi
    dx_e_vec{iu} = zeros([length(time),5]);% x_eベクトルの微分値
    F{iu} = zeros([length(time),7]); % 慣性座標系における"目標"座標，および曲率など
    X_I{iu} = zeros([length(time),5]); % 慣性座標位置における状態，および s
    dX_I{iu} = zeros([length(time),5]);% 慣性座標位置における状態の微分値
    V_g{iu} = zeros([length(time),1]); % 対地速度v_g
    phi{iu} = zeros([length(time),1]); % ロール角Φ
    p{iu} = zeros([length(time),1]); % ロール角速度p
    phi_r{iu} = zeros([length(time),1]); % 目標ロール角Φr
    dphi_r{iu} = zeros([length(time),1]); % ロール角速度dΦr
    delta_a{iu} = zeros([length(time),1]); % エルロン入力
    error{iu} = zeros([length(time),1]); % ヨー角と飛行経路角の差分の観測用
    GammaChi{iu} = zeros([length(time),2]);
    Cxi{iu} = zeros([length(time),7]);   % F_PATHへの入力の計算用
    ZV{iu} = zeros([length(time),1]); % 0ベクトル

    phi_r_f{iu}= zeros([length(time),1]);
    D_phi_r_f{iu} = zeros([length(time),1]);

    dchi_d{iu} = zeros([length(time),1]);

    dchi_d_f{iu} = zeros([length(time),1]);
    D_dchi_d_f{iu} = zeros([length(time),1]);

    delta_a_x{iu} = zeros([length(time),1]);
end

% 初期値の代入
X_I{1}(1,:) = X_I_0{1}(1:5,1)';   % 初期値の代入
X_I{2}(1,:) = X_I_0{2}(1:5,1)';   % 初期値の代入

Cxi{1}(1,:) = Cxi0{1};
Cxi{2}(1,:) = Cxi0{2};

phi_0{1} = 0;
phi_0{2} = 0;
p_0{1} = 0;
p_0{2} = 0;

phi{1}(1,1) = phi_0{1};
phi{2}(1,1) = phi_0{2};
p{1}(1,1) = p_0{1};
p{2}(1,1) = p_0{2};

%定数
g=9.80665;
rho=1.155; %ρ
Ss=0.42;
ww=2; 
d0=0.2393;     %δe^0
Ixx=0.1580;    

Mc=rho*Ss*ww*V_a^2*(0.1659*d0+0.2578)/2;

%% パラメータ
a = 40;
b = 0.8;
c = 0;

%% ゲイン
fp = 1.7;
fd = 0.15;

%% 計算（シミュレーション本体）%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% この中に基本式あり．いずれはちゃんと関数化すること

for i = 1 : length(time)
    
    %時間を表示
    t = i*dt
    
    %風の設定
    Wx = 3*sin(2*t);
    Wy = 3*sin(2*t);
    
    % Wx = 1;
    % Wy = 1;

    %慣性座標系における状態の取り出し．
    for iu = 1:2
        xI{iu} =X_I{iu}(i,1); %位置x[m]
        yI{iu} =X_I{iu}(i,2); %位置y[m]
        psi{iu}=X_I{iu}(i,3); %ヨー角[rad]
        s{iu} = X_I{iu}(i,4); %経路長s [m]
    %参照経路情報(現時点の経路情報)
        F{iu}(i,:) = F_PATH_FX79_r1(Cxi{iu}(i,:))';

    %参照経路情報(現時点の経路情報)の取り出し
        chi_d{iu} = F{iu}(i,3);  % 目標航路角[rad]
        kappa{iu} = F{iu}(i,5);  % 曲率[rad/m]
        xi{iu} = F{iu}(i,6);     % 媒介変数 xi（ζ）

    %慣性座標系におけるx,yの目標との偏差
        x_eI{iu} = (X_I{iu}(i,1:2) - F{iu}(i,1:2))';

        %対地速度Vgの計算
        %機体座標系{B} → 慣性座標系{I}の回転行列...?個人的な解釈は，V_aをベクトルにするための三角関数
        SyIB{iu}=[cos(psi{iu}) sin(psi{iu});  % ヨー回転
                 -sin(psi{iu}) cos(psi{iu})];
        
        % 対地速度ベクトル,x方向はそのままdot_x, y方向はそのままdot_yとなる
        % と思ったけど，後でdx，dyはなぜか再定義されてる．．．？
        V_g_vec{iu} = SyIB{iu}*[V_a; 0] + [Wx; Wy];  % [m/s]
        % 対地速度の大きさ（ノルム）
        V_g{iu}(i,1) = norm{iu}(V_g_vec{iu});  % [m/s]

        % 航路角χの導出
        chi{iu} = atan2(-V_g_vec{iu}(2),V_g_vec{iu}(1));  % [rad]

        
        %% 慣性座標系からセレ・フレネ座標系への変換
        %χeの補正
        x_e_vec{iu}(i,3) = -chi{iu} + chi_d{iu}; %χeの導出
                                    %chi - chi_dではない理由は，回転行列のsinの符号を揃えるため...?
        while (x_e_vec{iu}(i,3) < -pi || x_e_vec{iu}(i,3) >= pi)  % 0 <= chi_e < 2*pi の否定
            if (x_e_vec{iu}(i,3) > pi)
                x_e_vec{iu}(i,3) = x_e_vec(i,3) - 2*pi;
            elseif (x_e_vec{iu}(i,3) < -pi)
                x_e_vec{iu}(i,3) = x_e_vec{iu}(i,3) + 2*pi;
            end
        end

        GammaChi{iu}(i,1) = chi{iu};  % 航路角χの１ステップ前の保存用

        %慣性座標系{I}→セレ・フレネ座標系{F}の回転行列
        %ここはzの座標軸を機体下向きに取っているのでsinの符号が普通の機体とは逆．発狂．
        SyIF0{iu}=[cos(chi_d{iu}) -sin(chi_d{iu});
                   sin(chi_d{iu}) cos(chi_d{iu})];

        %慣性座標系におけるx,y偏差を，セレ・フレネ座標系における誤差xe,yeへ座標軸回転
        x_e_vec{iu}(i,1:2) = (SyIF0{iu}*x_eI{iu})';
        x_e_vec{iu}(i,4) = s{iu};   % s の代入
        x_e_vec{iu}(i,5) = xi{iu};   % xi(s)の値を取得

        %セレ・フレネ座標系における変数x_e_vecについて再定義．
        x_e{iu} = x_e_vec{iu}(i,1);  % [m]
        y_e{iu} = x_e_vec{iu}(i,2);  % [m]
        chi_e{iu} = x_e_vec{iu}(i,3);  % [rad]

        %% 目標ロール角の計算
        %目標ロール角の計算
        %A_Practical_Design_Approach_for_Complex_Path_Tracking_Controlの式26
        phi_r{iu}(i,1) = atan((V_g{iu}(i,1)/(a*g))*(b*(y_e{iu}+a*chi_e{iu})+V_g{iu}(i,1)*sin(chi_e{iu})-V_g{iu}(i,1)*kappa{iu}*x_e{iu}*cos(chi_e{iu})+a*dchi_d{iu}(i,1)-c*kappa{iu}*x_e{iu}^2));
        
        
        %% 目標ロール角にLPF , phi_r_fにはD_phi_rの積分値っぽいものが入ってる
        Tp = 0.4;
        if i == 1
            phi_r_f{iu}(i,1) = phi_r{iu}(i,1);
            D_phi_r_f{iu}(i,1) = 0;
        else
            D_phi_r_f{iu}(i,1) = (1/Tp)*(phi_r{iu}(i,1) - phi_r_f{iu}(i-1,1));
            phi_r_f{iu}(i,1) = D_phi_r_f{iu}(i,1)*dt + phi_r_f{iu}(i-1,1);
        end

        
        %（LPFなしの目標ロール角速度）, dphi_rはphi_rの疑似微分値が入ってる
        if i == 1
            dphi_r{iu}(i,1) = 0;
        else
            dphi_r{iu}(i,1) = (phi_r{iu}(i,1)-phi_r{iu}(i-1,1))/dt;
        end

    end

    %% エルロン入力の計算

    %（LPFなしのエルロン入力）
    % Pは
    % A_Practical_Design_Approach_for_Complex_Path_Tracking_Controlの式16
    % で出てくる．phi（ロール）の微分値(角速度)がp，あとで定義式出てくる．
    % このコントローラ自体は式26で出てくる．
    delta_a(i,1) = -fp*(phi(i,1)-phi_r(i,1))-fd*(p(i,1) - dphi_r(i,1));

    % エルロン入力の上限下限値を設定
    if delta_a(i,1)>=20*pi/180
        delta_a(i,1)=20*pi/180;
    elseif delta_a(i,1)<=-20*pi/180
        delta_a(i,1)=-20*pi/180;
    end

    %モデル式
    % A_Practical_Design_Approach_for_Complex_Path_Tracking_Controlの式18
    % c*x_eはコントローラ入力．高橋さんはu(t)と広く置いてる．
    % 微分値についてはdot_sみたいな記述とdx_eみたいな記述が多くてちょっと見づらい，統一したい
    dot_s = c*x_e +V_g(i,1)*cos(chi_e);

    %セレフレネ系の誤差ダイナミクス
    % A_Practical_Design_Approach_for_Complex_Path_Tracking_Controlの式12~14
    dx_e = V_g(i,1)*cos(chi_e)-dot_s*(1-kappa*y_e); 
    dy_e = V_g(i,1)*sin(chi_e)-dot_s*kappa*x_e;
    dchi_e = -g*tan(phi(i,1))/V_g(i,1) + dchi_d(i,1);

    % 詳細なパラメータについて定義してるだけ
    dphi = p(i,1);                                     %4
    dp = Mc*delta_a(i,1)/Ixx;                          %5

    % ここまで出てきた状態変数の時系列データ格納用
    x_e_vec(i,:) = [dx_e;dy_e;dchi_e;dot_s;0];

    % セレ・フレネ座標系と慣性座標系の間の角速度の変換
    dchi = - dchi_e + dchi_d(i,1) ;  % 航路角速度 [rad/s]
    GammaChi(i,2) = dchi;   % 保存用
    dpsi = V_g(i,1)*dchi / (V_a*cos(chi-psi));  % ヨー角速度 [rad/s]

    % キネマティクス式，v_g_vecとどう定儀式が違うのかよくわからない
    % V_g_vecをつかっても通ったのでヨシ
    % つまりはdx，dyをわかりやすく再定義してるだけ
    % dx = V_a*cos(psi) + Wx;
    % dy = -V_a*sin(psi) + Wy;
    dx = V_g_vec(1,1);
    dy = V_g_vec(2,1);

    % ここまでの変数の時系列データ格納用
    % dX_I(時刻,x速度；y速度；ヨー角速度；弧長速度) 
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
    

    phi(i+1,1) = phi(i,1) + dphi*dt;   % ロール角 Φ[rad]の更新
    
    % ロール角の上限下限値を設定
    if phi(i+1,1) > 70*pi/180
        phi(i+1,1) = 70*pi/180;
    elseif phi(i+1,1) < -70*pi/180
        phi(i+1,1) = -70*pi/180;
    end

    % phi（ロール）の微分値(角速度)pの更新
    p(i+1,1) =p(i,1) + dp*dt;
    
    % ロール角速度の上限下限値を設定
    if p(i+1,1) > 100*pi/180
        p(i+1,1) = 100*pi/180;
    elseif p(i+1,1) < -100*pi/180
        p(i+1,1) = -100*pi/180;
    end
    
    % xi 計算用の変数保存
    Cxi(i+1,:) = [s + dot_s*dt, dot_s, t, dt, s, F(i,6), F(i,7)];
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% シミュレーション本体ここまで
%% 以後はアニメーション設定なので，しっかり見る必要はなし．


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
