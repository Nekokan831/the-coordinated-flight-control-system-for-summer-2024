% 2022/3/30 by Yutoku Takahashi
% 2�������f���@��������
% 4���[��

% �_�����e�o�[�W����
% �f�[�^����

%% X,M1 �̒�`
[sA_x,sA_y] = size(A{1}); % A�s��̃T�C�Y�擾
[sB_x,sB_y] = size(B{1}); % B�s��̃T�C�Y�擾

[X,X_num,X_ch] = lmivar(1,[sA_x 1]) % X�̒�`
[Xc,Xc_num,Xc_ch] = lmivar(1,[1 1]) % Xc�̒�`

% ***������Ɖ��***
% lmivar�ł͒�`�����s�񂲂Ƃɏ��Ԃɔԍ�������U��D
% ����Ƃ͕ʂɁC�e�v�f���Ƃɂ����Ԃɔԍ�������U��D
% ��̏ꍇ�C��`�����s��X,Xc�ɂ��ꂼ��1,2������U����D
% X_ch,Xc_ch�ł��ꂼ��̍s��̏ڍׂ��ۑ�����C���̒��ɂ͗v�f���Ƃɔԍ�������U���ۑ������D
% X_num, Xc_num�͂��ꂼ��̗v�f�̍ŏI�ԍ����ۑ������D
% ���̕ӂ�̎d�l�����p���邱�Ƃōs��̗v�f���ƂɌʂ̐ݒ���s�����Ƃ��o����D
% ��̓I�ɂ�lmivar��3�Ōʐݒ�ł���D
% ���̍ۂ͊e�s��̗v�f���Ƃɔԍ����蓮�Őݒ肷��D�����ԍ��̗v�f�������l�ƂȂ�D

% F_s�ɂ��Đ��` �� F_x, F_chi �Ɍʂ̓��͐�����^���邽�ߊe�v�f���ʂɒ�`����D
M11_num = Xc_num+3;
Mchi11_num = Xc_num+3;
% �e���[��i���ƂɁC�P�̂̃Q�C��M11���`����D
for i=1:m_num
    [M11{i},M11_num,M11_ch{i}] = lmivar(3,[Xc_num+1 Xc_num+2 Xc_num+3 % M_s�͑S���[���ŋ��ʂ̔ԍ�������U��
                                M11_num+1 M11_num+2 M11_num+3]) % M_chi�̓��[�����ɕʂ̒l������U��  
                           
    % �ȉ��́C���͐����p�̒�`
    % M11�̊e�s���Ƃ�LMI�ŏ�����ݒ肵�������߁C�ʂɒ�`
    % ��̓I�ɂ́CM11{i}=[Mx11{i},Mchi11{i}]^T 
    [Ms11{i},~,Ms11_ch{i}] = lmivar(3,[Xc_num+1 Xc_num+2 Xc_num+3])
    [Mchi11{i},Mchi11_num,Mchi11_ch{i}] = lmivar(3,[Mchi11_num+1 Mchi11_num+2 Mchi11_num+3])
    
end

M12_num = M11_num+1;
Mchi12_num = M11_num+1;
% �e���[��i���ƂɁCM12���`����D
for i=1:m_num
    [M12{i},M12_num,M12_ch{i}] = lmivar(3,[M11_num+1
                                M12_num+1]) % M_x �ɂ��Ă͑S���[���œ��ꂷ�邽�ߌʂɒ�`
    [Ms12{i},~,Ms12_ch{i}] = lmivar(3,M11_num+1) % ���͐����p M_x,M_chi��p��
    [Mchi12{i},Mchi12_num,Mchi12_ch{i}] = lmivar(3,Mchi12_num+1) % ���͐����p M_x,M_chi��p��                            
end

[M22,~,M22_ch] = lmivar(3,Mchi12_num+1) % ���͐����p M_x,M_chi��p��

% eps_fs = 10^(-6);

%% LMI�̏�����
% lmi_cont�Ƀ}�C�i�X������E�E�E0���傫��

% �P�� �������
% X > 0
lmi_count=newlmi;
lmiterm([-lmi_count 1 1 X],1,1);
lmiterm([-lmi_count 2 2 Xc],1,1);

% �Q�� �������
% lambda�`�` > 0
lmi_count=newlmi;
lmiterm([-lmi_count 1 1 0], lambda);
lmiterm([-lmi_count 2 1 0], x0);
lmiterm([-lmi_count 3 1 0], x0_c);
lmiterm([-lmi_count 2 2 X],1,1);
lmiterm([-lmi_count 3 3 Xc],1,1);

% �R�� �������
% AX + X^T A^T - BM- B^T M^T�`�` < 0
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
%     % �R�� ���͐����̏���
%     % [1 x0^T; x0 X] > 0
%     lmi_count=newlmi;
%     lmiterm([-lmi_count 1 1 0], 1);
%     lmiterm([-lmi_count 2 1 0], x0);
%     lmiterm([-lmi_count 3 1 0], x0_c);
%     lmiterm([-lmi_count 2 2 X], 1, 1);
%     lmiterm([-lmi_count 3 3 Xc], 1, 1);

% �S�� ���͐����̏���
% [X M_s^T; M_s mu] > 0
    for i=1:m_num
        lmi_count=newlmi;
        lmiterm([-lmi_count 1 1 X], 1, 1);
        lmiterm([-lmi_count 2 2 Xc], 1, 1);
        lmiterm([-lmi_count 3 1 Ms11{i}], 1, 1);
        lmiterm([-lmi_count 3 2 Ms12{i}], 1, 1);
        lmiterm([-lmi_count 3 3 0], mu_s^2/lambda);
    end

% �T�� ���͐����̏���
% [X M_chi^T; M_chi mu] > 0
    for i=1:m_num
        lmi_count=newlmi;
        lmiterm([-lmi_count 1 1 X], 1, 1);
        lmiterm([-lmi_count 2 2 Xc], 1, 1);
        lmiterm([-lmi_count 3 1 Mchi11{i}], 1, 1);
        lmiterm([-lmi_count 3 2 Mchi12{i}], 1, 1);
        lmiterm([-lmi_count 3 3 0], mu_chi^2/lambda);
    end

% �U�� ���͐����̏���
% [X M_v^T; M_v mu] > 0
    for i=1:m_num
        lmi_count=newlmi;
        lmiterm([-lmi_count 1 1 X], 1, 1);
        lmiterm([-lmi_count 2 2 Xc], 1, 1);
        lmiterm([-lmi_count 3 2 M22], 1, 1);
        lmiterm([-lmi_count 3 3 0], mu_v^2/lambda);
    end
end

%% LMI�̌v�Z
lmisys=getlmis;   % LMI�̌v�Z
[tmin, xopt]=feasp2(lmisys);   % LMI�̉��@tmin < 0 �Ȃ��

% P1,P2,M1,N2,F,L �����ꂼ�ꓱ�o
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
    F11{lmi_num}=M11{lmi_num}/X;   % �t�B�[�h�o�b�N�Q�C���̌v�Z
    Fs11{lmi_num}=Ms11{lmi_num}/X;
    Fchi11{lmi_num}=Mchi11{lmi_num}/X;
    F12{lmi_num}=M12{lmi_num}/Xc;
    Fs12{lmi_num}=Ms12{lmi_num}/Xc;
    Fchi12{lmi_num}=Mchi12{lmi_num}/Xc;    
    F22{lmi_num}=M22/Xc;
end

% �{�V�X�e���̊e�s����`����D
X_C = [X zeros([sA_x,1]);
      zeros([1,sA_y]) Xc];
for i = 1:m_num
    M_C{i}=[M11{i} M12{i};0,0,0,M22];
    Ms_C{i}=[Ms11{i} Ms12{i}];
    Mchi_C{i}=[Mchi11{i} Mchi12{i}];
    Mv_C{i}=[0,0,0,M22];
end

%% LMI���������ꍇ�̍Čv�Z
% �ŗL�l���v�Z����D
eig_num = 1;
    
if tmin < 0 % ���̏ꍇ
    % �P�� �������
    eig_lmi1{1}=eig(X_C);   % �ŗL�lX�̗�x�N�g��
    eig_lmi_check(eig_num)=-min(eig_lmi1{1});
    eig_num = eig_num + 1;
    
    % �Q�� �������
    for i=1:m_num
        eig_lmi2{i}=eig([A_C{i}*X_C+(A_C{i}*X_C)'-B_C{i}*M_C{i}-(B_C{i}*M_C{i})']);
    end
    for i=1:m_num
        eig_lmi2_tempmax(i)=max(eig_lmi2{i});
    end
    eig_lmi_check(eig_num)=max(eig_lmi2_tempmax);
    eig_num = eig_num + 1;
    
    if Input_constraints == 1
        % �R�� ���͐����̏���
        eig_lmi3{1}=eig([lambda x0_C';x0_C X_C]);
        eig_lmi_check(eig_num)=-min(eig_lmi3{1});
        eig_num = eig_num + 1;

        % �S�� ���͐����̏���
        for i=1:m_num
            eig_lmi4{i}=eig([X_C Ms_C{i}';Ms_C{i} mu_s^2/lambda]);
        end
        for i=1:m_num
            eig_lmi4_tempmax(i)=-min(eig_lmi4{i});
        end
        eig_lmi_check(eig_num)=max(eig_lmi4_tempmax);
        eig_num = eig_num + 1;

        % �T�� ���͐����̏���
        for i=1:m_num
            eig_lmi5{i}=eig([X_C Mchi_C{i}';Mchi_C{i} mu_chi^2/lambda]);
        end
        for i=1:m_num
            eig_lmi5_tempmax(i)=-min(eig_lmi5{i});
        end
        eig_lmi_check(eig_num)=max(eig_lmi5_tempmax);
        eig_num = eig_num + 1;

        % �U�� ���͐����̏���
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

%% �o��
if tmin < 0 && flag_lmi< 0    % feasible
    disp('%%%%% ���I%%%%%');
    flag_feas = 1;
    lambda_up =lambda
elseif tmin < 0
    disp('%%%%% �Čv�Z�ł���...(�v���O�����ɖ�肪����\��)%%%%%');
    lambda_low = lambda
else
    disp('%%%%% ����D�D�D%%%%%');
    lambda_low = lambda
end
