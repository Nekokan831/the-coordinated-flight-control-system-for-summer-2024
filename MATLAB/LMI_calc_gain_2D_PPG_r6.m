% 2022/3/30 by Yutoku Takahashi
% 2次元モデル　協調制御
% 4ルール

% 論文投稿バージョン
% データ整理

%% X,M1 の定義
[sA_x,sA_y] = size(A{1}); % A行列のサイズ取得
[sB_x,sB_y] = size(B{1}); % B行列のサイズ取得

[X,X_num,X_ch] = lmivar(1,[sA_x 1]) % Xの定義
[Xc,Xc_num,Xc_ch] = lmivar(1,[1 1]) % Xcの定義

% ***ちょっと解説***
% lmivarでは定義した行列ごとに順番に番号を割り振る．
% それとは別に，各要素ごとにも順番に番号を割り振る．
% 上の場合，定義した行列X,Xcにそれぞれ1,2が割り振られる．
% X_ch,Xc_chでそれぞれの行列の詳細が保存され，この中には要素ごとに番号が割り振られ保存される．
% X_num, Xc_numはそれぞれの要素の最終番号が保存される．
% この辺りの仕様を活用することで行列の要素ごとに個別の設定を行うことが出来る．
% 具体的にはlmivarの3で個別設定できる．
% その際は各行列の要素ごとに番号を手動で設定する．同じ番号の要素＝同じ値となる．

% F_sについて線形 ＆ F_x, F_chi に個別の入力制限を与えるため各要素を個別に定義する．
M11_num = Xc_num+3;
Mchi11_num = Xc_num+3;
% 各ルールiごとに，単体のゲインM11を定義する．
for i=1:m_num
    [M11{i},M11_num,M11_ch{i}] = lmivar(3,[Xc_num+1 Xc_num+2 Xc_num+3 % M_sは全ルールで共通の番号を割り振る
                                M11_num+1 M11_num+2 M11_num+3]) % M_chiはルール毎に別の値を割り振る  
                           
    % 以下は，入力制限用の定義
    % M11の各行ごとにLMIで条件を設定したいため，個別に定義
    % 具体的には，M11{i}=[Mx11{i},Mchi11{i}]^T 
    [Ms11{i},~,Ms11_ch{i}] = lmivar(3,[Xc_num+1 Xc_num+2 Xc_num+3])
    [Mchi11{i},Mchi11_num,Mchi11_ch{i}] = lmivar(3,[Mchi11_num+1 Mchi11_num+2 Mchi11_num+3])
    
end

M12_num = M11_num+1;
Mchi12_num = M11_num+1;
% 各ルールiごとに，M12を定義する．
for i=1:m_num
    [M12{i},M12_num,M12_ch{i}] = lmivar(3,[M11_num+1
                                M12_num+1]) % M_x については全ルールで統一するため個別に定義
    [Ms12{i},~,Ms12_ch{i}] = lmivar(3,M11_num+1) % 入力制限用 M_x,M_chiを用意
    [Mchi12{i},Mchi12_num,Mchi12_ch{i}] = lmivar(3,Mchi12_num+1) % 入力制限用 M_x,M_chiを用意                            
end

[M22,~,M22_ch] = lmivar(3,Mchi12_num+1) % 入力制限用 M_x,M_chiを用意

% eps_fs = 10^(-6);

%% LMIの条件式
% lmi_contにマイナスをつける・・・0より大きい

% １つ目 安定条件
% X > 0
lmi_count=newlmi;
lmiterm([-lmi_count 1 1 X],1,1);
lmiterm([-lmi_count 2 2 Xc],1,1);

% ２つ目 安定条件
% lambda～～ > 0
lmi_count=newlmi;
lmiterm([-lmi_count 1 1 0], lambda);
lmiterm([-lmi_count 2 1 0], x0);
lmiterm([-lmi_count 3 1 0], x0_c);
lmiterm([-lmi_count 2 2 X],1,1);
lmiterm([-lmi_count 3 3 Xc],1,1);

% ３つ目 安定条件
% AX + X^T A^T - BM- B^T M^T～～ < 0
for i=1:m_num
    lmi_count=newlmi;
    lmiterm([lmi_count 1 1 X], A{i}, 1, 's');
%    lmiterm([lmi_count 2 2 Xc], 0, 0, 's');
    lmiterm([lmi_count 1 1 M11{i}], B{i}, -1, 's');
    lmiterm([lmi_count 1 2 M12{i}], B{i}, -1);
    lmiterm([lmi_count 2 2 M22], 1, -1, 's');
    lmiterm([lmi_count 3 1 X],1,1);
    lmiterm([lmi_count 4 2 Xc],1,1);
    lmiterm([lmi_count 3 3 0],-inv(W));
    lmiterm([lmi_count 4 4 0],-inv(W_c));
    lmiterm([lmi_count 5 1 M11{i}], -1, 1);
    lmiterm([lmi_count 5 2 M12{i}], -1, 1);
    lmiterm([lmi_count 6 2 M22], -1, 1);
    lmiterm([lmi_count 5 5 0],-inv(R));
    lmiterm([lmi_count 6 6 0],-inv(R_c));    
end

if Input_constraints == 1
%     % ３つ目 入力制限の条件
%     % [1 x0^T; x0 X] > 0
%     lmi_count=newlmi;
%     lmiterm([-lmi_count 1 1 0], 1);
%     lmiterm([-lmi_count 2 1 0], x0);
%     lmiterm([-lmi_count 3 1 0], x0_c);
%     lmiterm([-lmi_count 2 2 X], 1, 1);
%     lmiterm([-lmi_count 3 3 Xc], 1, 1);

% ４つ目 入力制限の条件
% [X M_s^T; M_s mu] > 0
    for i=1:m_num
        lmi_count=newlmi;
        lmiterm([-lmi_count 1 1 X], 1, 1);
        lmiterm([-lmi_count 2 2 Xc], 1, 1);
        lmiterm([-lmi_count 3 1 Ms11{i}], 1, 1);
        lmiterm([-lmi_count 3 2 Ms12{i}], 1, 1);
        lmiterm([-lmi_count 3 3 0], mu_s^2/lambda);
    end

% ５つ目 入力制限の条件
% [X M_chi^T; M_chi mu] > 0
    for i=1:m_num
        lmi_count=newlmi;
        lmiterm([-lmi_count 1 1 X], 1, 1);
        lmiterm([-lmi_count 2 2 Xc], 1, 1);
        lmiterm([-lmi_count 3 1 Mchi11{i}], 1, 1);
        lmiterm([-lmi_count 3 2 Mchi12{i}], 1, 1);
        lmiterm([-lmi_count 3 3 0], mu_chi^2/lambda);
    end

% ６つ目 入力制限の条件
% [X M_v^T; M_v mu] > 0
    for i=1:m_num
        lmi_count=newlmi;
        lmiterm([-lmi_count 1 1 X], 1, 1);
        lmiterm([-lmi_count 2 2 Xc], 1, 1);
        lmiterm([-lmi_count 3 2 M22], 1, 1);
        lmiterm([-lmi_count 3 3 0], mu_v^2/lambda);
    end
end

%% LMIの計算
lmisys=getlmis;   % LMIの計算
[tmin, xopt]=feasp2(lmisys);   % LMIの解　tmin < 0 なら可解

% P1,P2,M1,N2,F,L をそれぞれ導出
X=dec2mat(lmisys,xopt,X);
Xc=dec2mat(lmisys,xopt,Xc);

for lmi_num=1:m_num
    M11{lmi_num}=dec2mat(lmisys,xopt,M11{lmi_num});
    Ms11{lmi_num}=dec2mat(lmisys,xopt,Ms11{lmi_num});
    Mchi11{lmi_num}=dec2mat(lmisys,xopt,Mchi11{lmi_num});
    M12{lmi_num}=dec2mat(lmisys,xopt,M12{lmi_num});
    Ms12{lmi_num}=dec2mat(lmisys,xopt,Ms12{lmi_num});
    Mchi12{lmi_num}=dec2mat(lmisys,xopt,Mchi12{lmi_num});
end
    M22=dec2mat(lmisys,xopt,M22);
for lmi_num=1:m_num
    F11{lmi_num}=M11{lmi_num}/X;   % フィードバックゲインの計算
    Fs11{lmi_num}=Ms11{lmi_num}/X;
    Fchi11{lmi_num}=Mchi11{lmi_num}/X;
    F12{lmi_num}=M12{lmi_num}/Xc;
    Fs12{lmi_num}=Ms12{lmi_num}/Xc;
    Fchi12{lmi_num}=Mchi12{lmi_num}/Xc;    
    F22{lmi_num}=M22/Xc;
end

% 本システムの各行列を定義する．
X_C = [X zeros([sA_x,1]);
      zeros([1,sA_y]) Xc];
for i = 1:m_num
    M_C{i}=[M11{i} M12{i};0,0,0,M22];
    Ms_C{i}=[Ms11{i} Ms12{i}];
    Mchi_C{i}=[Mchi11{i} Mchi12{i}];
    Mv_C{i}=[0,0,0,M22];
end

%% LMIが解けた場合の再計算
% 固有値を計算する．
eig_num = 1;
    
if tmin < 0 % 可解の場合
    % １つ目 安定条件
    eig_lmi1{1}=eig(X_C);   % 固有値Xの列ベクトル
    eig_lmi_check(eig_num)=-min(eig_lmi1{1});
    eig_num = eig_num + 1;
    
    % ２つ目 安定条件
    for i=1:m_num
        eig_lmi2{i}=eig([A_C{i}*X_C+(A_C{i}*X_C)'-B_C{i}*M_C{i}-(B_C{i}*M_C{i})']);
    end
    for i=1:m_num
        eig_lmi2_tempmax(i)=max(eig_lmi2{i});
    end
    eig_lmi_check(eig_num)=max(eig_lmi2_tempmax);
    eig_num = eig_num + 1;
    
    if Input_constraints == 1
        % ３つ目 入力制限の条件
        eig_lmi3{1}=eig([lambda x0_C';x0_C X_C]);
        eig_lmi_check(eig_num)=-min(eig_lmi3{1});
        eig_num = eig_num + 1;

        % ４つ目 入力制限の条件
        for i=1:m_num
            eig_lmi4{i}=eig([X_C Ms_C{i}';Ms_C{i} mu_s^2/lambda]);
        end
        for i=1:m_num
            eig_lmi4_tempmax(i)=-min(eig_lmi4{i});
        end
        eig_lmi_check(eig_num)=max(eig_lmi4_tempmax);
        eig_num = eig_num + 1;

        % ５つ目 入力制限の条件
        for i=1:m_num
            eig_lmi5{i}=eig([X_C Mchi_C{i}';Mchi_C{i} mu_chi^2/lambda]);
        end
        for i=1:m_num
            eig_lmi5_tempmax(i)=-min(eig_lmi5{i});
        end
        eig_lmi_check(eig_num)=max(eig_lmi5_tempmax);
        eig_num = eig_num + 1;

        % ６つ目 入力制限の条件
        for i=1:m_num
            eig_lmi6{i}=eig([X_C Mv_C{i}';Mv_C{i} mu_v^2/lambda]);
        end
        for i=1:m_num
            eig_lmi6_tempmax(i)=-min(eig_lmi6{i});
        end
        eig_lmi_check(eig_num)=max(eig_lmi6_tempmax);
        eig_num = eig_num + 1;
    end
    
    flag_lmi=max(eig_lmi_check);
end

%% 出力
if tmin < 0 && flag_lmi< 0    % feasible
    disp('%%%%% 可解！%%%%%');
    flag_feas = 1;
    lambda_up =lambda
elseif tmin < 0
    disp('%%%%% 再計算できず...(プログラムに問題がある可能性)%%%%%');
    lambda_low = lambda
else
    disp('%%%%% 非可解．．．%%%%%');
    lambda_low = lambda
end
