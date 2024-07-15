function Fxi = F_PATH_FX79_r1(Cxi)
    %% プログラムの使い方，説明
    % calc_PATH_2D_PPG_r0.m で得られた結果を張り付けて使用する．．
    % １．「xiの計算」項目に xi もしくは dPdxi の得られている方を張り付ける．
    % ２．「目標経路」項目にFxiを張り付ける．
    % 
    % 返値: Fxi -----------------------------
    % 1列目 [ 目標 x 座標（慣性座標系）[m]，
    % 2列目   目標 y 座標（慣性座標系）[m], 
    % 3列目   目標航路角 [rad/s],
    % 4列目   目標航路角速度,
    % 5列目   曲率 kappa [rad/m],
    % 6列目   媒介変数 xi,
    % 7列目   現時点での i の実数値]
    % ----------------------------------------
    %% xi 計算の前処理
    
    % Cxi(i+1,:)
    % =[s + dot_s*dt, dot_s, t, dt, s, F(i,6), F(i,7)];  
    % 引数
    s = Cxi(1);  % 経路長 [m]
    dot_s = Cxi(2);  % 経路長の時間微分 [m/s]  (NO NEED)
    t = Cxi(3);  % 時刻 [s]  (NO NEED)
    dt = Cxi(4);  % 微小時間(更新周期) [s]
    s_old = Cxi(5);  % 1ステップ前の経路長 [m]
    xi_old = Cxi(6);  % 1ステップ前のxi(s)
    i_sav = Cxi(7);  % 1ステップ前のi
    xi = -10^5;   % 条件分岐のためあり得ない値を初期値に設定している．
    
    if s == 0 % s = 0で 0 割りが発生する場合あり，微小値を代入
        s = 10^(-8);
    end
    
    dxi = 10^(-7);  % dxi の分解能の設定
    
    s_calc = s_old; %１ステップ前の s の値
    i = i_sav; % １ステップ前の i の値
    
    %% xi(論文中ではzeta(s)) の計算 sからxiを求める
    while 1
        
    % 直線経路: 2つのWPで定義-----------------
    %     P0 = [0 0];
    %     P1 = [500 100];
    %     dist_WPs = sqrt(sum((P0 - P1).^2));
    %     xi = s/dist_WPs;
    %----------------------------------------
     
    
    %円　★
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % xi =s/70;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %r=60の8の字 ★
    %  dPdxi =120*(cos(dxi*i)^2 + cos(2*dxi*i)^2)^(1/2);
    
    
    %四葉曲線 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % r=200;
    %  dPdxi =(2^(1/2)*r*(3*cos(4*dxi*i) + 5)^(1/2))/2;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %レムニスケート経路%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    r=120;
    dPdxi =r/(2 - sin(dxi*i)^2)^(1/2);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %dPdxi = 100*(4*cos(xi)^4 + 1)^(1/2);
    %dPdxi =120*(cos(dxi*i)^2 + cos(2*dxi*i)^2)^(1/2);
    
        % % % xi の一般解が得られていない場合の xi の計算．
        if xi < -10^3
            % % 以下，xi の計算
                s_calc = s_calc + dPdxi * dxi;
                if s_calc >= s
                    xi = i*dxi;
                    i_sav = i;
                    break;
                end
            i = i+1;
        else
            break;
        end
     end
    
     if xi == 0 % xi = 0で 0 割りが発生する場合あり，微小値を代入
         xi = 10^(-8);
     end
     
     dot_xi =(xi - xi_old)/dt;
    
    % %% 目標経路 
    % % Pixhawk2 へ実装する経路-------------------------------
    % % 直線経路: 2つのWPで定義
    % x_d_local = (1 - xi)*P0(1) + xi*P1(1);
    % y_d_local = (1 - xi)*P0(2) + xi*P1(2);
    % chi_d_local = atan2((P1(2) - P0(2)), (P1(1) - P0(1)));
    % dchi_d_local = 0;
    % kappa_local = 0;
    % Fxi = [x_d_local, y_d_local, chi_d_local, dchi_d_local, kappa_local, xi, i_sav];
    % %------------------------------------------------------
    
    
    
    
    %r=70の円
    % Fxi = [ 70*cos(xi), 70*sin(xi), atan2(-cos(xi), -sin(xi)), -dot_xi, 1/70, xi, i_sav;];
    
    
    %% 
    
    %左旋回(+0)★
    %Fxi = [ 60*cos(xi), 60*sin(xi), atan2(-cos(xi), -sin(xi)), -dot_xi, 1/60, xi, i_sav;];
    
    
    %%
    %右旋回(+pi/2) ★
    %Fxi = [ 70*sin(xi), 70*cos(xi), atan2(sin(xi), cos(xi)), dot_xi, 1/70, xi, i_sav;];
    
    %r=60の8の字 lis ★
    %  Fxi =[120*sin(xi), 60*sin(2*xi), atan2(-cos(2*xi), cos(xi)), (dot_xi*(3*sin(xi) - 2*sin(xi)^3))/(4*sin(xi)^4 - 5*sin(xi)^2 + 2), -(abs(sin(xi))*(2*sin(xi)^2 - 3))/(120*(4*cos(xi)^4 - 3*cos(xi)^2 + 1)^(3/2)), xi, i_sav];
    
    
    
    %四葉曲線
    
    %Fxi =[-r*sin(2*xi)*sin(xi), r*sin(2*xi)*cos(xi), atan2(-r*cos(xi)*(3*cos(xi)^2 - 2), r*sin(xi)*(3*sin(xi)^2 - 2)), -(dot_xi*(3*cos(4*xi) + 13))/(3*cos(4*xi) + 5), (2^(1/2)*(3*cos(4*xi) + 13))/(r*(3*cos(4*xi) + 5)^(3/2)), xi, i_sav];
    
    %レムニスケート経路%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Fxi =[(r*sin(xi))/(cos(xi)^2 + 1), (r*cos(xi)*sin(xi))/(cos(xi)^2 + 1), atan2((r*(3*sin(xi)^2 - 2))/(sin(xi)^2 - 2)^2, -(r*cos(xi)*(cos(xi)^2 - 3))/(cos(xi)^2 + 1)^2), -(3*dot_xi*sin(xi))/(sin(xi)^2 - 2), (3*abs(sin(xi)))/(r*(cos(xi)^2 + 1)^(1/2)), xi, i_sav];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %リサージュ
    %Fxi = [120*sin(xi), 60*sin(2*xi), atan2(-cos(2*xi), cos(xi)), (dot_xi*(3*sin(xi) - 2*sin(xi)^3))/(4*sin(xi)^4 - 5*sin(xi)^2 + 2), -(abs(sin(xi))*(2*sin(xi)^2 - 3))/(120*(4*cos(xi)^4 - 3*cos(xi)^2 + 1)^(3/2)), xi, i_sav];
    
    %Fxi =[200*sin(xi), 50*sin(2*xi), atan2(-cos(2*xi)/2, cos(xi)), (dot_xi*(6*sin(xi) - 4*sin(xi)^3))/(4*sin(xi)^4 - 8*sin(xi)^2 + 5), -(abs(sin(xi))*(2*sin(xi)^2 - 3))/(50*(4*cos(xi)^4 + 1)^(3/2)), xi, i_sav];