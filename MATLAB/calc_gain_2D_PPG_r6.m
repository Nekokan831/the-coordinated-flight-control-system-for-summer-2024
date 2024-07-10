% 2022/3/30 by Yutoku Takahashi
% 2�������f���@��������
% 4���[��

% �_�����e�o�[�W����
% �f�[�^����

% �œK����̒ǉ�

%% ������
close all
clear
clc

tic % �v�Z���Ԃ̌v���J�n
%% ���f���p�����[�^�̐ݒ�
Input_constraints = 1; % ���͐��� 1:����, 0:�Ȃ�

% Vg_min = 5;     % �Βn���x �ŏ��l[m/s]
% Vg_max = 25;    % �Βn���x �ő�l[m/s]

Vg_min = 20;     % �@�̑��x �ŏ��l[m/s]
Vg_max = 50;    % �@�̑��x �ő�l[m/s]
V0 = 35;   % �΋C���x

epsilon1 = sin(178*pi/180) / (178*pi/180);   % sin chi �̍ŏ��X��

% kappa_max = 0.05;
% U_x_max = 15;   % ���͐���
% mu_s = 15;
% mu_chi = 90*pi/180;
% mu_v = 10;

% kappa_max = 0.05;
% U_x_max = 45;   % ���͐���
% mu_s = 3000;
% mu_chi = 18000*pi/180;
% mu_v = 2000;

kappa_max = 0.05;
U_x_max = 100;   % ���͐���
mu_s = 100;
mu_chi = 180*pi/180;
mu_v = 30;

D_kappa_max = (Vg_max+U_x_max)*kappa_max;    % �@�̑��x �ő�l[m/s]
D_kappa_min = -D_kappa_max;

x0=[10;10;pi];   % �����l
% x0=[10;10;90*pi/180];   % �����l
x0_c=30;   % �����l
% x0=[100;100;900*pi/180];   % �����l
% x0_c=100;   % �����l
% x0=[0;0;0];   % �����l
% x0_c=0;   % �����l

% % % % �ŏ�������lambda�ɂ���
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

%% �s���`
% LMI��F�ɓ���Ȑ������K�v�Ȋ֌W��C�����ł͒P�̂̃��f���̖���`
% LMI�����Ŋg��n���L�q

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

% ���`�����f��
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

m_num = length(A);   % �����o�V�b�v�֐��̐�
%% ��������p�@�g��n�̋L�q
[sA_x,sA_y] = size(A{1}); % A�s��̃T�C�Y�擾
[sB_x,sB_y] = size(B{1}); % B�s��̃T�C�Y�擾

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

%% ���䐫�̊m�F
Mc = [B_C{1} A_C{1}*B_C{1} A_C{1}^2*B_C{1} A_C{1}^3*B_C{1} A_C{1}^4*B_C{1}];
RANK = rank(Mc);

if RANK == length(A_C{1})
    disp('����!!')
else
    disp('�����D�D�D�ꎞ��~���܂��D�D�D')
    pause(1000)
end

%% LMI�̐ݒ�p�����[�^

%% LMI�̌v�Z�@(�ȉ��͕ύX�s�v)
flag_feas = 0;   % ���Ȃ� 1 �ɂȂ�D

count_for_optimization = 0;
% setlmis([]);   % LMI�̏�����
% LMI_calc_gain_2D_PPG_r6;
while (lambda_up-lambda_low)>distance_lambda
% % %     �񐔂��v��
    count_for_optimization = count_for_optimization + 1;
% % %     2���@
    lambda=(lambda_up+lambda_low)/2
    
    setlmis([]);   % LMI�̏�����
    LMI_calc_gain_2D_PPG_r6;
end
%% ����ꂽlambda �ōČv�Z
lambda = lambda_up;
setlmis([]);   % LMI�̏�����
LMI_calc_gain_2D_PPG_r6;

%% �t�B�[�h�o�b�N�Q�C���̏o��

if flag_feas == 1 % ��������ꂽ�ꍇ
    % �t�B�[�h�o�b�N�Q�C���̏o��
    
    for i = 1:m_num
%        disp(['F',num2str(i)])
        F{i}=[F11{i} F12{i};0,0,0,F22{i}];
    end
    
    % ���� F - [F_x,F_chi] ���v�Z
    for i = 1:m_num
        F_ch{i} = F{i}-[Fs11{i},Fs12{i};Fchi11{i},Fchi12{i};0,0,0,F22{i}];
    end
    for ii = 1:sB_y+1
        F_gosa(ii) = 0;
        for i = 1:m_num
            F_gosa(ii) = F_gosa(ii) + F_ch{i}(ii,:)*F_ch{i}(ii,:)';
        end
    end
    
    % �덷���������ꍇ�ɏo��
    if F_gosa(1) ~= 0 || F_gosa(2) ~= 0 || F_gosa(3) ~= 0
        disp('�Q�C�����������v�Z�ł��ĂȂ��C�덷����')
        F_gosa(1)
        F_gosa(2)
        F_gosa(3)
    else
        disp('�Čv�ZOK!!')
    end
    
end
M_C{i}
    for i = 1:m_num
       disp(['F',num2str(i)])
        F{i}
    end

lambda

toc % �v�Z���Ԃ̌v���I��





