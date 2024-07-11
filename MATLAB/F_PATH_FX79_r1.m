function Fxi = F_PATH_FX79_r1(Cxi)
%% �v���O�����̎g�����C����
% calc_PATH_2D_PPG_r0.m �œ���ꂽ���ʂ𒣂�t���Ďg�p����D�D
% �P�D�uxi�̌v�Z�v���ڂ� xi �������� dPdxi �̓����Ă�����𒣂�t����D
% �Q�D�u�ڕW�o�H�v���ڂ�Fxi�𒣂�t����D
% 
% �Ԓl: Fxi -----------------------------
% 1��� [ �ڕW x ���W�i�������W�n�j[m]�C
% 2���   �ڕW y ���W�i�������W�n�j[m], 
% 3���   �ڕW�q�H�p [rad/s],
% 4���   �ڕW�q�H�p���x,
% 5���   �ȗ� kappa [rad/m],
% 6���   �}��ϐ� xi,
% 7���   �����_�ł� i �̎����l]
% ----------------------------------------
%% xi �v�Z�̑O����

% Cxi(i+1,:)
% =[s + dot_s*dt, dot_s, t, dt, s, F(i,6), F(i,7)];  
% ����
s = Cxi(1);  % �o�H�� [m]
dot_s = Cxi(2);  % �o�H���̎��Ԕ��� [m/s]  (NO NEED)
t = Cxi(3);  % ���� [s]  (NO NEED)
dt = Cxi(4);  % ��������(�X�V����) [s]
s_old = Cxi(5);  % 1�X�e�b�v�O�̌o�H�� [m]
xi_old = Cxi(6);  % 1�X�e�b�v�O��xi(s)
i_sav = Cxi(7);  % 1�X�e�b�v�O��i
xi = -10^5;   % ��������̂��߂��蓾�Ȃ��l�������l�ɐݒ肵�Ă���D

if s == 0 % s = 0�� 0 ���肪��������ꍇ����C�����l����
    s = 10^(-8);
end

dxi = 10^(-7);  % dxi �̕���\�̐ݒ�

s_calc = s_old; %�P�X�e�b�v�O�� s �̒l
i = i_sav; % �P�X�e�b�v�O�� i �̒l

%% xi(�_�����ł�zeta(s)) �̌v�Z s����xi�����߂�
while 1
    
% �����o�H: 2��WP�Œ�`-----------------
%     P0 = [0 0];
%     P1 = [500 100];
%     dist_WPs = sqrt(sum((P0 - P1).^2));
%     xi = s/dist_WPs;
%----------------------------------------
 

%�~�@��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% xi =s/70;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%r=60��8�̎� ��
%  dPdxi =120*(cos(dxi*i)^2 + cos(2*dxi*i)^2)^(1/2);


%�l�t�Ȑ� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% r=200;
%  dPdxi =(2^(1/2)*r*(3*cos(4*dxi*i) + 5)^(1/2))/2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%�����j�X�P�[�g�o�H%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r=120;
dPdxi =r/(2 - sin(dxi*i)^2)^(1/2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%dPdxi = 100*(4*cos(xi)^4 + 1)^(1/2);
%dPdxi =120*(cos(dxi*i)^2 + cos(2*dxi*i)^2)^(1/2);

    % % % xi �̈�ʉ��������Ă��Ȃ��ꍇ�� xi �̌v�Z�D
    if xi < -10^3
        % % �ȉ��Cxi �̌v�Z
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

 if xi == 0 % xi = 0�� 0 ���肪��������ꍇ����C�����l����
     xi = 10^(-8);
 end
 
 dot_xi =(xi - xi_old)/dt;

% %% �ڕW�o�H 
% % Pixhawk2 �֎�������o�H-------------------------------
% % �����o�H: 2��WP�Œ�`
% x_d_local = (1 - xi)*P0(1) + xi*P1(1);
% y_d_local = (1 - xi)*P0(2) + xi*P1(2);
% chi_d_local = atan2((P1(2) - P0(2)), (P1(1) - P0(1)));
% dchi_d_local = 0;
% kappa_local = 0;
% Fxi = [x_d_local, y_d_local, chi_d_local, dchi_d_local, kappa_local, xi, i_sav];
% %------------------------------------------------------




%r=70�̉~
%Fxi = [ 70*cos(xi), 70*sin(xi), atan2(-cos(xi), -sin(xi)), -dot_xi, 1/70, xi, i_sav;];


%% 

%������(+0)��
%Fxi = [ 60*cos(xi), 60*sin(xi), atan2(-cos(xi), -sin(xi)), -dot_xi, 1/60, xi, i_sav;];


%%
%�E����(+pi/2) ��
%Fxi = [ 70*sin(xi), 70*cos(xi), atan2(sin(xi), cos(xi)), dot_xi, 1/70, xi, i_sav;];

%r=60��8�̎� lis ��
%  Fxi =[120*sin(xi), 60*sin(2*xi), atan2(-cos(2*xi), cos(xi)), (dot_xi*(3*sin(xi) - 2*sin(xi)^3))/(4*sin(xi)^4 - 5*sin(xi)^2 + 2), -(abs(sin(xi))*(2*sin(xi)^2 - 3))/(120*(4*cos(xi)^4 - 3*cos(xi)^2 + 1)^(3/2)), xi, i_sav];



%�l�t�Ȑ�

%Fxi =[-r*sin(2*xi)*sin(xi), r*sin(2*xi)*cos(xi), atan2(-r*cos(xi)*(3*cos(xi)^2 - 2), r*sin(xi)*(3*sin(xi)^2 - 2)), -(dot_xi*(3*cos(4*xi) + 13))/(3*cos(4*xi) + 5), (2^(1/2)*(3*cos(4*xi) + 13))/(r*(3*cos(4*xi) + 5)^(3/2)), xi, i_sav];

%�����j�X�P�[�g�o�H%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 Fxi =[(r*sin(xi))/(cos(xi)^2 + 1), (r*cos(xi)*sin(xi))/(cos(xi)^2 + 1), atan2((r*(3*sin(xi)^2 - 2))/(sin(xi)^2 - 2)^2, -(r*cos(xi)*(cos(xi)^2 - 3))/(cos(xi)^2 + 1)^2), -(3*dot_xi*sin(xi))/(sin(xi)^2 - 2), (3*abs(sin(xi)))/(r*(cos(xi)^2 + 1)^(1/2)), xi, i_sav];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%���T�[�W��
%Fxi = [120*sin(xi), 60*sin(2*xi), atan2(-cos(2*xi), cos(xi)), (dot_xi*(3*sin(xi) - 2*sin(xi)^3))/(4*sin(xi)^4 - 5*sin(xi)^2 + 2), -(abs(sin(xi))*(2*sin(xi)^2 - 3))/(120*(4*cos(xi)^4 - 3*cos(xi)^2 + 1)^(3/2)), xi, i_sav];

%Fxi =[200*sin(xi), 50*sin(2*xi), atan2(-cos(2*xi)/2, cos(xi)), (dot_xi*(6*sin(xi) - 4*sin(xi)^3))/(4*sin(xi)^4 - 8*sin(xi)^2 + 5), -(abs(sin(xi))*(2*sin(xi)^2 - 3))/(50*(4*cos(xi)^4 + 1)^(3/2)), xi, i_sav];