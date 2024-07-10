% sim_2D_PPG_r11.m
% 2022/3/31 by Yutoku Takahashi
% 2�������f���@��������
% 4���[��

% �_�����e�o�[�W����
% �f�[�^����
%% _/_/_/_/_/_/  ����!!  _/_/_/_/_/_/
% �֐� F_PATH_PPG_r** �ɖڕW�o�H���L�q���邱��!!
%_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

%% �����ݒ�Ə�����
% ������
clc
clear
close all

% �t�H���g�ƕ����T�C�Y�̐ݒ�
% set(0,'defaultAxesFontSize',15)   %�����l�� 9
set(0,'defaultTextFontSize',15)   %�����l�� 9

%% �p�����[�^�C�ڕW�o�H�Ȃǂ̐ݒ�
% % % % �V�~�����[�V�����E�A�j���[�V�������ݒ� % % % %
% �V�~�����[�V�������Ԑݒ�
dt = 0.1;   % �V�~�����[�V���� �������
% time = 0:dt:80;   % �V�~�����[�V��������
% time = 0:dt:0.1;   % �V�~�����[�V��������
time = 0:dt:360;   % �V�~�����[�V��������
% time = 0:dt:80;   % �V�~�����[�V��������

% �A�j���[�V�����ݒ�
anime_ON = 0;   % �A�j���[�V�����́@�I���E�I�t 0:�Ȃ��C1:�ЂƂC2:�����Ȋp�x
SS = 5;   % �A�j���[�V�������x�C�v���b�g�̊Ԋu
PPG_size = 0.5;   % �A�j���[�V�����̎���PPG�̑傫��

fig_num = 8; % �O���t�̖���

% �A�j���[�V�����̎��_
% az_a = -37.5;   %���ʊp(3���������ݒ�� -37.5)
% el_a = 30;   %�p(3���������ݒ�� 30)
az_a = 0;   %���ʊp(3���������ݒ�� -37.5)
el_a = 90;   %�p(3���������ݒ�� 30)

% �F�ݒ�x�N�g��
CV = [0 0 1; % ��
    1 0 0; % ��
    0.9290 0.6940 0.1250; % �I�����W
    0 0 0; % ��
    0 1 1]; % ���F 

% % % % UAV������ԁE���O���ݒ� % % % %
% �������W�n�ɂ�����ڕW���W����̕΍�
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

% % % % �t�B�[�h�o�b�N�Q�C���Q�C���ݒ� % % % %
% % �R�X�g�ۏ؁C���͐����Ȃ�
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

% �R�X�g�ۏ؁C���͐�������
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

% �Q�l�����Ŏg�p���Ă���UAV�p�̃Q�C��
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

% % % % UAV�̐ݒ� % % % %
V0 = 15;   % �΋C���x
V_min=5;     % �@�̑��x �ŏ��l[m/s]
V_max=25;     % �@�̑��x �ő�l[m/s]
% V_min=10;     % �@�̑��x �ŏ��l[m/s]
% V_max=20;     % �@�̑��x �ő�l[m/s]
mu_x = 15;   % ���͐���

% V0 = 28;   % �΋C���x
% V0 = 35;   % �΋C���x
% V_min=20;     % �@�̑��x �ŏ��l[m/s]
% V_max=50;     % �@�̑��x �ő�l[m/s]
% mu_x = 45;   % ���͐���

k_max = 1.3;
k_min = 0.7;

% �t�@�W�B���͈̔�
epsilon1 = sin(178*pi/180) / (178*pi/180);
kappa_max = 0.05;
D_kappa_max = (V_max+mu_x)*kappa_max;    % �@�̑��x �ő�l[m/s]
D_kappa_min = -D_kappa_max;

% �� �̐ݒ�
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

% �ڕW�o�H�Ɋւ���ݒ�
time_c1=120; % �؂�ւ����ԂP
time_c2=240; % �؂�ւ����ԂQ

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

%% �ȉ���{�I�ɑS����

%% ������Ԃ̌v�Z
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

num_UAV = length(x_I_0); % UAV�̑䐔
for iu = 1:num_UAV
    Cxi0{iu} = [s0{iu}, 0, 0, dt, s0{iu}, 0, 0, iu, eta{iu},s0{iu}]; % F_PATH �֗^������͂̏������
    X_I_0{iu} = F_PATH_PPG_r3(Cxi0{iu})'+[x_I_0{iu}, s0{iu}, 0, 0, 0]'; % �������W�n�ɂ����鏉�����
%     X_I_0{iu} = [x_I_0{iu}, s0{iu}, 0, 0, 0]'; % �������W�n�ɂ����鏉�����
%     X_I_0{iu} = [x_I_0{iu}, 30*(iu-1), 0, 0, 0]'; % �������W�n�ɂ����鏉�����
%     X_I_0{iu} = [x_I_0{iu}, error0{iu}, 0, 0, 0]'; % �������W�n�ɂ����鏉�����
%     X_I_0{iu} = [x_I_0{iu}, 0, 0, 0, 0]'; % �������W�n�ɂ����鏉�����
end

%% ���O���蓖��
% �����ʒu�C�������x
for iu = 1:num_UAV
    x_e_vec{iu} = zeros([length(time),5]); % �덷�_�C�i�~�N�X�̏�ԗ�x_e�x�N�g���C����� s, xi
    dx_e_vec{iu} = zeros([length(time),6]);% x_e�x�N�g���̔����l
    F{iu} = zeros([length(time),7]); % �������W�n�ɂ�����ڕW���W�C����ыȗ��Ȃ�
    X_I{iu} = zeros([length(time),5]); % �������W�ʒu�ɂ������ԁC����� s
    dX_I{iu} = zeros([length(time),5]);% �������W�ʒu�ɂ������Ԃ̔����l
    M1{iu} = zeros([length(time),1]);
    M2{iu} = zeros([length(time),1]);
    N1{iu} = zeros([length(time),1]);
    N2{iu} = zeros([length(time),1]);
    u_vec{iu} = zeros([length(time),3]); % ���͗�U�x�N�g��
    V{iu} = zeros([length(time),1]); % �΋C���x v
    Vg{iu} = zeros([length(time),1]); % �Βn���x v_g
    error{iu} = zeros([length(time),1]); % ���[�p�Ɣ�s�o�H�p�̍����̊ϑ��p
    Chi_save{iu} = zeros([length(time),2]);
    Cxi{iu} = zeros([length(time),10]);   % F_PATH�ւ̓��͂̌v�Z�p
    Cxi{iu}(1,2)=V0;
    % �������
    X_I{iu}(1,:) = X_I_0{iu}(1:5,1)';   % �����l�̑��
    Cxi{iu}(1,:) = Cxi0{iu};
    J_sum{iu}=0;
end
ZV = zeros([length(time),1]); % 0�x�N�g��

%% ���͗ʁC�ڕW�O���⊵�����W�ʒu�̌v�Z
for i = 1:length(time)
    % ���Ԃ�\��
    t = (i-1)*dt

% eta�̓r���؂�ւ������i�_���ɂ͎g�p�����j%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    
% Step1: x,y,chi,s,Delta�̃f�[�^�擾 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ave_s_eta = 0; % s_i/eta�̕��ϒl
    for iu = 1:num_UAV
        ave_s_eta=ave_s_eta+X_I{iu}(i,4)/eta{iu};
    end
    ave_s_eta = ave_s_eta/num_UAV;
    
    for iu = 1:num_UAV
        % % % % % �������W�ɂ������Ԃ̎��o�� % % % % %
        xI{iu} = X_I{iu}(i,1);
        yI{iu} = X_I{iu}(i,2);
        psi{iu} = X_I{iu}(i,3);
        s{iu} = X_I{iu}(i,4);
        Delta{iu}(i,1) = X_I{iu}(i,4)/eta{iu} - ave_s_eta; % ���R
        
        % ��]�s��̌v�Z
        % �@�̍��W�n�̊������W�n�̕ϊ��s��
        SyIB{iu}=[cos(psi{iu}) sin(psi{iu});   %���[��]
            -sin(psi{iu}) cos(psi{iu})];
        
        if i > 1.5 % 1�X�e�b�v�O�̒l���g�p
            Vg_vec = SyIB{iu}*[V{iu}(i-1,1);0]+[Wx;Wy];   % �������W�n�̑��x�ɕ��𑫂�
            chi{iu} = atan2(-Vg_vec(2),Vg_vec(1));   % �������W�n
        else
            Vg_vec = SyIB{iu}*[15;0]+[Wx;Wy];   % �������W�n�̑��x�ɕ��𑫂�
            chi{iu} = atan2(-Vg_vec(2),Vg_vec(1));   % �������W�n
        end
        Chi_save{iu}(i,1) = chi{iu}; % chi�̎��Ԑ��ڂ�ۑ�
    end
% Step2: �Z���t���l���W�̃f�[�^�擾 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for iu = 1:num_UAV
        F{iu}(i,:) = F_PATH_PPG_r3(Cxi{iu}(i,:))'; % �������W�n�ɂ�����ڕW�ʒu
        kappa{iu} = F{iu}(i,5); % �ȗ�
        
        % ���ʂ̌v�Z
        x_e_vec{iu}(i,3) = - chi{iu} + F{iu}(i,3); % �Z���t���l���W�n
        while (x_e_vec{iu}(i,3) < -pi || x_e_vec{iu}(i,3) >= pi) % 0 <= chi_e < 2*pi �̔ے�
            if (x_e_vec{iu}(i,3) > pi)
                x_e_vec{iu}(i,3) = x_e_vec{iu}(i,3) - 2*pi;
            elseif (x_e_vec{iu}(i,3) < -pi)
                x_e_vec{iu}(i,3) = x_e_vec{iu}(i,3) + 2*pi;
            end
        end
        % % ���i�ʒu�̌v�Z
        x_eI{iu} = (X_I{iu}(i,1:2) - F{iu}(i,1:2))';   % �������W�n�ɂ�����ڕW�ʒu�Ƃ̍�

        % �������W�n�̃Z���t���l���W�n�̕ϊ��s��
        SyIF0{iu}=[cos(F{iu}(i,3)) -sin(F{iu}(i,3));   %���[��]
            sin(F{iu}(i,3)) cos(F{iu}(i,3))];
        x_e_vec{iu}(i,1:2) = (SyIF0{iu}*x_eI{iu})';
        x_e_vec{iu}(i,4) = Delta{iu}(i,1);
        x_e_vec{iu}(i,5) = s{iu};   % s �̑��
        x_e_vec{iu}(i,6) = F{iu}(i,6);   % xi(s)�̒l���擾
        
        % �Z���t���l���W�n�ɂ�����ʒu�C�p��
        x_e{iu} = x_e_vec{iu}(i,1);
        y_e{iu} = x_e_vec{iu}(i,2);
        chi_e{iu} = x_e_vec{iu}(i,3);
    end
% Step3: u_s, u_v�̌v�Z %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for iu = 1:num_UAV
        u_s{iu} = - Fg{1}(1,:) * x_e_vec{iu}(i,1:4)';
        u_v{iu}(i,1) = -Fg{1}(3,:)*x_e_vec{iu}(i,1:4)'; % ���P
    end
% Step4: �Βn���x����ё΋C���x�̌v�Z %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    for iu = 1:num_UAV
        Vg{iu}(i,1) = (eta{iu}*(V0 + u_v{iu}(i,1))-u_s{iu})/cos(chi_e{iu});

        % �΋C���x�̌v�Z
        % �L�l�}�e�B�N�X�̎����t�Z����D
        % sin or cos �̒l���������Ȃ�ƌ덷���傫���Ȃ�̂ŁC�傫�����ŏꍇ����
        if psi{iu} < 135*pi/180 && psi{iu}>-135*pi/180 && abs(psi{iu})>45*pi/180
            V{iu}(i,1) = (Vg{iu}(i,1)*sin(chi{iu})+Wy)/sin(psi{iu});
        else
            V{iu}(i,1) = (Vg{iu}(i,1)*cos(chi{iu})-Wx)/cos(psi{iu});
        end
        
        % �΋C���x�͈͂��O�ꂽ�ꍇ�̏���
        if V{iu}(i,1) > V_max % �͈͉z���̏ꍇ
            V{iu}(i,1) = V_max; % �΋C���x���ő�l�ŃT�`��悤�ɐݒ�
            % �Βn���x�Ɠ��͗ʂ��Čv�Z
            if psi{iu} < 135*pi/180 && psi{iu}>-135*pi/180 && abs(psi{iu})>45*pi/180
                Vg{iu}(i,1)=(V{iu}(i,1)*sin(psi{iu})-Wy)/sin(chi{iu});
            else
                Vg{iu}(i,1)=(V{iu}(i,1)*cos(psi{iu})+Wx)/cos(chi{iu});
            end
            u_s{iu} = eta{iu}*(V0-u_v{iu}(i,1))-Vg{iu}(i,1)*cos(chi_e{iu}); % Vg�C�����̌덷��u_s�ŋz��
        elseif  V{iu}(i,1) < V_min % �͈͉z���̏ꍇ
             V{iu}(i,1) = V_min; % �΋C���x���ŏ��l�ŃT�`��悤�ɐݒ�
             % �Βn���x���Čv�Z
            if psi{iu} < 135*pi/180 && psi{iu}>-135*pi/180 && abs(psi{iu})>45*pi/180
                Vg{iu}(i,1)=(V{iu}(i,1)*sin(psi{iu})-Wy)/sin(chi{iu});
            else
                Vg{iu}(i,1)=(V{iu}(i,1)*cos(psi{iu})+Wx)/cos(chi{iu});
            end
            u_s{iu} = eta{iu}*(V0-u_v{iu}(i,1))-Vg{iu}(i,1)*cos(chi_e{iu}); % Vg�C�����̌덷��u_s�ŋz��
        end
        if Vg{iu}(i,1) > V_max+(Wx^2+Wy^2)^(1/2) % �͈͉z���̏ꍇ
            Vg{iu}(i,1) = V_max+(Wx^2+Wy^2)^(1/2);
        elseif Vg{iu}(i,1) < V_min-(Wx^2+Wy^2)^(1/2)
            Vg{iu}(i,1) = V_min-(Wx^2+Wy^2)^(1/2);
        end
    end
% Step5: u_chi�̌v�Z %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
    for iu = 1:num_UAV
        % �����o�V�b�v�֐��̌v�Z
        if chi_e{iu} == 0 % chi_e��0���̑Ώ�
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

        % �������
        gain{iu}=0;
        for ii = 1:length(Fg)
            gain{iu} = gain{iu} + h{iu}(i,ii) * Fg{ii};            
        end
        u_vec{iu}(i,:) = - gain{iu} * x_e_vec{iu}(i,1:4)';

        check(i,1)= u_vec{iu}(i,1)-u_s{iu}; % ���`�R���g���[���ƃt�@�W�B�R���g���[���Ɍ덷���Ȃ����m�F
        check(i,2)= u_vec{iu}(i,3)-u_v{iu}(i,1); % ���`�R���g���[���ƃt�@�W�B�R���g���[���Ɍ덷���Ȃ����m�F
        u_vec{iu}(i,1)=u_s{iu}; % ���x���T�`��Ƃ�u_s�Ō덷���z�����Ă��邽�ߍX�V
    end
    
% Step6: dot_s, dpsi�̌v�Z %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for iu = 1:num_UAV
        dot_s{iu} = u_vec{iu}(i,1) + Vg{iu}(i,1)*cos(chi_e{iu});
        % �Z���E�t���l���W�n�Ɗ������W�n�̊Ԃ̊p���x�̕ϊ�
        dchi{iu} = -u_vec{iu}(i,2)+F{iu}(i,4);
        dpsi{iu} = Vg{iu}(i,1)*dchi{iu} / (V{iu}(i,1)*cos(chi{iu}-psi{iu}));
%         % FX_79 �ő���񑬓x 90[deg./s]�Ɖ���
        if dpsi{iu} > 90 * pi / 180
            dpsi{iu} = 90 * pi / 180;
        elseif dpsi{iu} < - 90 * pi / 180
            dpsi{iu} = -90 * pi / 180;
        end

% % %         % �Q�l�����p�@���񑬓x�̏���l��ݒ�
%         if dpsi{iu} > 0.54
%             dpsi{iu} = 0.54;
%         elseif dpsi{iu} < - 0.54
%             dpsi{iu} = -0.54;
%         end
    end

%Step7: �ʒu�f�[�^�̍X�V (���@�̏ꍇ��s�̏��̂ݍX�V) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ave_ds_eta = 0;   % dot_Delta�̕��ϒl�̌v�Z
    for iu = 1:num_UAV
        ave_ds_eta=ave_ds_eta+dot_s{iu}/eta{iu};
    end
    ave_ds_eta = ave_ds_eta/num_UAV;
    
    for iu = 1:num_UAV
        % �������W�n�ɂ�����e�ϐ��̔����l�̌v�Z
        dx{iu} = V{iu}(i,1)*cos(psi{iu})+Wx;
        dy{iu} = -V{iu}(i,1)*sin(psi{iu})+Wy;
        
        dX_I{iu}(i,1:4) = [dx{iu};dy{iu};dpsi{iu};dot_s{iu}];

        % �������W�n�ɂ������Ԃ̍X�V
        X_I{iu}(i+1,1) = X_I{iu}(i,1) + dx{iu}*dt;
        X_I{iu}(i+1,2) = X_I{iu}(i,2) + dy{iu}*dt;
        X_I{iu}(i+1,3) = X_I{iu}(i,3) + dpsi{iu}*dt;
        while (X_I{iu}(i+1,3) < -pi || X_I{iu}(i+1,3) >= pi) % 0 <= chi_e < 2*pi �̔ے�
            if (X_I{iu}(i+1,3) > pi)
                X_I{iu}(i+1,3) = X_I{iu}(i+1,3) - 2*pi;
            elseif (X_I{iu}(i+1,3) < -pi)
                X_I{iu}(i+1,3) = X_I{iu}(i+1,3) + 2*pi;
            end
        end
        X_I{iu}(i+1,4) = X_I{iu}(i,4) + dot_s{iu}*dt;
        % �o�H�v�Z�p�ɕϐ����܂Ƃ߂����̂̍X�V
        Cxi{iu}(i+1,:) = [s{iu} + dot_s{iu}*dt, dot_s{iu}, t+dt, dt, s{iu}, F{iu}(i,6), F{iu}(i,7), iu, eta{iu}, s0{iu}];
    end

%���܂�: �Z���t���l���W�n�ɂ�����e��ԕϐ��̔����l�̌v�Z %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �V�~�����[�V�����ɂ͕s�g�p
% �f�[�^�̕ۑ��̂�
    for iu = 1:num_UAV
        dx_e{iu} = Vg{iu}(i,1)*cos(chi_e{iu}) - dot_s{iu}*(1-kappa{iu}*y_e{iu});
        dy_e{iu} = Vg{iu}(i,1)*sin(chi_e{iu}) - dot_s{iu}*kappa{iu}*x_e{iu};
        dchi_e{iu} = u_vec{iu}(i,2);
        dDelta{iu} = dot_s{iu}/eta{iu} - ave_ds_eta;
        
        dx_e_vec{iu}(i,:) = [dx_e{iu};dy_e{iu};dchi_e{iu};dDelta{iu};dot_s{iu};0];  % ��ԕϐ��̑��x
    end
    
    for iu = 1:num_UAV % �]���l�̌v�Z
        J_sum{iu} = J_sum{iu} + (x_e_vec{iu}(i,1:4)*x_e_vec{iu}(i,1:4)'+u_vec{iu}(i,:)*u_vec{iu}(i,:)')*dt;
%         J_sum{iu} = J_sum{iu} + (x_e_vec{iu}(i,1:4)*x_e_vec{iu}(i,1:4)'+u_vec{iu}(i,:)*[1*10^4 0 0;0 1*10^8 0; 0 0 1*10^3]*u_vec{iu}(i,:)')*dt;
    end
end

% % % % % ���ʂȃf�[�^������ % % % % %
for iu = 1:num_UAV
    X_I{iu}(end,:) = [];
    J_sum{iu} % �]���l�̕\��
end

%% �v�Z���ʂ̃v���b�g
% % ��ԕϐ��̎��Ԑ���
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

title('�ʒ�����ьʒ����x�̎��Ԑ���')
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
%title('��ԕϐ��̎��Ԑ���')
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

% % ���͗ʂ̎��Ԑ���
figure('Position',[650 380 570 250])
subplot(3,1,1)
grid on
hold on
for iu = 1:num_UAV
    plot(time,u_vec{iu}(:,1),'color',CV(iu,:))
end
title('���͗ʂ̎��Ԑ���')
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

% % �������W�ʒu�C�p���̎��Ԑ���
figure('Position',[50 50 570 250])
subplot(3,1,1)
grid on
hold on
for iu = 1:num_UAV
    plot(time,X_I{iu}(:,1),'color',CV(iu,:))
end
title('�������W�ʒu�C�p���̎��Ԑ���')
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

% % �^�̑Βn���x�Ɠ��͑��x
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
%title('�΋C���x�C�Βn���x')
set( gca, 'FontName','Times','FontSize',10 );
ylabel('V_g[m/s]', 'FontName','Times','FontSize',12)
xlabel('time[s]', 'FontName','Times','FontSize',12)

% % �����o�[�V�b�v�֐��̎��Ԑ���
figure('Position',[1950 -100 750 700])
subplot(2,1,1)
grid on
hold on
for iu = 1:num_UAV
    plot(time,M1{iu},'color',CV(iu,:))
end
title('�����o�V�b�v�֐�')
ylabel('K1')

subplot(2,1,2)
grid on
hold on
for iu = 1:num_UAV
    plot(time,N1{iu},'color',CV(iu,:))
end
ylabel('M1')
xlabel('time[sec.]')

%% �R������s�O��
% figure
% set(gca,'YDir','reverse')
% grid on
% hold on
% plot3(F{1}(:,2),F{1}(:,1),ZV,'g','linewidth',2)
% for iu = 1:num_UAV
%     plot3(X_I{iu}(:,2),X_I{iu}(:,1),ZV,'color',CV(iu,:),'linewidth',1)
% end
% axis equal
% title('�ڕW�o�H�Ɣ�s�O��')
% legend('�ڕW�O��')
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
%     title('�ڕW�o�H�Ɣ�s�O��')
%     legend('�ڕW�o�H','��s�O��')
%     xlabel('y[m]')
%     ylabel('x[m]')
%     zlabel('z[m]')
%     az = 180;
%     el = 90;
%     view(az, el)
% end

%% �A�j���[�V��������
if anime_ON == 1
    SS = SS*0.1;
elseif anime_ON == 2
    SS = SS*0.2;
end

%% �A�j���[�V����
end_time = time(end);
time = time';
min_X = 0;
min_Y = 0;
max_X = 0;
max_Y = 0;
id = 0;
for iu = 1:num_UAV
    % ���x����
    Xanime_n{iu} = [X_I{iu}(:,1:2),ZV,ZV,X_I{iu}(:,3:4),ZV,F{iu}(:,1:2),ZV];

    for n = 1:10
        Xanime{iu}(:,n) = interp1(time,Xanime_n{iu}(:,n),0:SS:end_time);
    end
    t_3(:,1) = interp1(time,time,0:SS:end_time);

    % �f�[�^�͈�
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
%�A�j���[�V��������
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

        % �ڕW�o�H�̐��̃v���b�g
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
        % ��s�O�Ղ̐��̃v���b�g
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

        % % ����Œ�
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