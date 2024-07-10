function Fxi = F_PATH_PPG_r3(Cxi)
%% �v���O�����̎g�����C����
% calc_PATH_2D_PPG_r0.m �œ���ꂽ���ʂ𒣂�t���Ďg�p����D�D
% �P�D�uxi�̌v�Z�v���ڂ� xi �������� dPdxi �̓����Ă�����𒣂�t����D
% �Q�D�u�ڕW�o�H�v���ڂ�Fxi�𒣂�t����D
% 
% �Ȃ��CFxi�̒��g�C����я��Ԃ͎��̒ʂ�D
% 1��� [ �ڕW���� x ���W�C
% 2���   �ڕW���� y ���W, 
% 3���   �ڕW�q�H�p psi,
% 4���   �ڕW�q�H�p���x,
% 5���   �ȗ� kappa,
% 6���   xi,
% 7���   �����_�ł� i �̎����l]
% 
%% xi �v�Z�̑O����
s = Cxi(1);
dot_s = Cxi(2);
t = Cxi(3);
dt = Cxi(4);
s_old = Cxi(5);
xi_old = Cxi(6);
i_sav = Cxi(7);
iu = Cxi(8);
k = Cxi(9);
s0 = Cxi(10);
xi = -10^5;   % ��������̂��߂��蓾�Ȃ��l�������l�ɐݒ肵�Ă���D

d_xi = 0;
% if t < 120
%     d_xi = 0;
% elseif 120 <= t && t < 240
%     d_xi = iu-1;
% else
%     if iu == 1
%         d_xi = 0;
%     elseif iu == 2 || iu == 4
%         d_xi = 1;
%     else
%         d_xi = 2;
%     end
% end

% % % % if iu == 1
% % % %     d_xi = 0;
% % % % else
% % % %     d_xi = 2;
% % % % end


if s == 0 % s = 0�� 0 ���肪��������ꍇ����C�����l����
    s = 10^(-6);
end

dxi = 10^(-5);  % dxi �̕���\�̐ݒ�

s_calc = s_old; %�P�X�e�b�v�O�� s �̒l
i = i_sav; % �P�X�e�b�v�O�� i �̒l

%% xi �̌v�Z
while 1
% % % % % xi �̈�ʉ��������Ă���ꍇ�C�����ɋL�q
%     xi = s;    % ����
%     xi = s/100;   % �~

%     xi = s/(100*k);
%     xi = (s - s0)/(100*k);

%     xi = (15^(1/2)*s^(1/2))/15;    % �C���{�����[�g�Ȑ�

% % % % % xi �̈�ʉ��������Ă��Ȃ��ꍇ�C������ dPdxi ���L�q�D
    % dPdxi = 15*2^(1/2)*(5 - 3*cos(2*dxi*i))^(1/2); % �ȉ~
    % dPdxi = 40*(13*sin(dxi*i)^2 + 36)^(1/2); % �ȉ~
    
    % if iu == 1
    %     dPdxi = 20*2^(1/2)*(85 - 13*cos(2*dxi*i + 47/50))^(1/2);
    % elseif iu == 2
    %     dPdxi = 40*(13*sin(dxi*i)^2 + 36)^(1/2);
    % elseif iu == 3
    %     dPdxi = 20*2^(1/2)*(85 - 13*cos(2*dxi*i + 281/25))^(1/2);
    % else
    %     dPdxi = 40*(13*sin(dxi*i + 643/125)^2 + 36)^(1/2);    
    % end
    
    dPdxi = 50*k*(5*sin(dxi*i)^2 + 4)^(1/2); % �ȉ~�@PPG
    
%     dPdxi = 10*(25*cos(dxi*i)^2 + 4)^(1/2); % sin�g
%     dPdxi = ; % ���T�[�W���Ȑ��o�H�C8�̎��ia=1, b=2�j
%     dPdxi = 50*(9*cos(3*dxi*i)^2 + 16*cos(4*dxi*i)^2)^(1/2); % ���T�[�W���Ȑ��o�H,�ia=3, b=4�j
%     dPdxi = 25*2^(1/2)*(3*cos(4*dxi*i) + 5)^(1/2); % ���t�Ȑ��̌o�H 4.1254e+04
%     dPdxi = 150/(sin(dxi*i)^2 + 1)^(1/2); % �����j�X�P�[�g�o�H

%     dPdxi = 50*(25*sin(5*dxi*i)^2 + 36*sin(6*dxi*i)^2)^(1/2); % ���T�[�W���Ȑ��o�H�C�d�ʑ�

% % % xi �̈�ʉ��������Ă��Ȃ��ꍇ�� xi �̌v�Z�D
    if xi < -10^3
        % % �ȉ��Cxi �̌v�Z
            s_calc = s_calc + dPdxi * dxi;
            if s_calc >= s
                xi = i*dxi;
                i_sav=i;
                break
            end
        i = i+1;
    else
        break;
    end
end

if xi == 0 % xi = 0�� 0 ���肪��������ꍇ����C�����l����
    xi = 10^(-8);
end

dot_xi =(xi-xi_old)/dt;

%% �ڕW�o�H 
% ����
% Fxi = [ xi, 0, 0, 0, 0, xi, i_sav];
% �~
% Fxi = [ 100*cos(xi), 100*sin(xi), atan2(-cos(xi), -sin(xi)), -dot_xi, 1/100, xi, i_sav];
% Fxi = [ 100*cos(iu/10 - xi), -100*sin(iu/10 - xi), atan2(-cos(iu/10 - xi), sin(iu/10 - xi)), -dot_xi, 1/100, xi, i_sav];

% Fxi = [ 100*k*cos(d_xi/10 - xi), -100*k*sin(d_xi/10 - xi), atan2(-k*cos(d_xi/10 - xi), k*sin(d_xi/10 - xi)), -dot_xi, 1/(100*k), xi, i_sav];

% �ȉ~
% Fxi = [ 60*cos(xi), 30*sin(xi), atan2(-cos(xi)/2, -sin(xi)), -(2*dot_xi)/(3*sin(xi)^2 + 1), 1/(15*(3*sin(xi)^2 + 1)^(3/2)), xi, i_sav]; 

% �ȉ~�@��r�p
% Fxi = [ 280*cos(xi), 240*sin(xi), atan2(-(6*cos(xi))/7, -sin(xi)), -(42*dot_xi)/(13*sin(xi)^2 + 36), 21/(20*(13*sin(xi)^2 + 36)^(3/2)), xi, i_sav];
% if iu == 1
%     Fxi = [ 280*cos(xi + 47/100), 240*sin(xi + 47/100), atan2(-(6*cos(xi + 47/100))/7, -sin(xi + 47/100)), -(42*dot_xi)/(13*sin(xi + 47/100)^2 + 36), 21/(20*(13*sin(xi + 47/100)^2 + 36)^(3/2)), xi, i_sav];
% elseif iu == 2
%     Fxi = [ 280*cos(xi), 240*sin(xi), atan2(-(6*cos(xi))/7, -sin(xi)), -(42*dot_xi)/(13*sin(xi)^2 + 36), 21/(20*(13*sin(xi)^2 + 36)^(3/2)), xi, i_sav];
% elseif iu == 3
%     Fxi = [ 280*cos(xi + 281/50), 240*sin(xi + 281/50), atan2(-(6*cos(xi + 281/50))/7, -sin(xi + 281/50)), -(42*dot_xi)/(13*sin(xi + 281/50)^2 + 36), 21/(20*(13*sin(xi + 281/50)^2 + 36)^(3/2)), xi, i_sav];
% else
%     Fxi = [ 280*cos(xi + 643/125), 240*sin(xi + 643/125), atan2(-(6*cos(xi + 643/125))/7, -sin(xi + 643/125)), -(42*dot_xi)/(13*sin(xi + 643/125)^2 + 36), 21/(20*(13*sin(xi + 643/125)^2 + 36)^(3/2)), xi, i_sav];
% end

% �ȉ~�@PPG
Fxi = [ 150*k*cos(xi), 100*k*sin(xi), atan2(-(2*k*cos(xi))/3, -k*sin(xi)), -(6*dot_xi)/(5*sin(xi)^2 + 4), 3/(25*k*(5*sin(xi)^2 + 4)^(3/2)), xi, i_sav];

% sin�g
% Fxi = [ 20*xi, 50*sin(xi), -atan((5*cos(xi))/2), -(10*dot_xi*sin(xi))/(25*sin(xi)^2 - 29), abs(sin(xi))/(25*cos(xi)^2 + 4)^(3/2), xi, i_sav];
% ���T�[�W���Ȑ��o�H�C8�̎��ia=1, b=2�j

% ���T�[�W���Ȑ��o�H�C8�̎��ia=3, b=4�j
% Fxi = [ 50*sin(3*xi), 50*sin(4*xi), atan2(-(4*cos(4*xi))/3, cos(3*xi)), (12*dot_xi*(sin(7*xi) + 7*sin(xi)))/(9*cos(6*xi) + 16*cos(8*xi) + 25), (6*2^(1/2)*abs(sin(7*xi) + 7*sin(xi)))/(25*(9*cos(6*xi) + 16*cos(8*xi) + 25)^(3/2)), xi, i_sav];
% % % �C���{�����[�g�Ȑ��̌o�H
% Fxi = [ 30*cos(xi) + 30*xi*sin(xi), 30*sin(xi) - 30*xi*cos(xi), atan2(-xi*sin(xi), xi*cos(xi)), -dot_xi, 1/(30*xi), xi, i_sav];
% % % ���t�Ȑ��̌o�H
% Fxi = [ 50*sin(2*xi)*cos(xi), 50*sin(2*xi)*sin(xi), atan2(sin(xi)*(3*sin(xi)^2 - 2), cos(xi)*(3*cos(xi)^2 - 2)), -(dot_xi*(3*cos(4*xi) + 13))/(3*cos(4*xi) + 5), (2^(1/2)*(3*cos(4*xi) + 13))/(50*(3*cos(4*xi) + 5)^(3/2)), xi, i_sav];
% % % �����j�X�P�[�g�o�H
% Fxi = [ (80*cos(xi))/(sin(xi)^2 + 1), (80*cos(xi)*sin(xi))/(sin(xi)^2 + 1), angle(((240*sin(xi)^2 - 80)*1i)/(80*(sin(xi)^2 + 1)^2) + (sin(xi)*(sin(xi)^2 - 3))/(sin(xi)^2 + 1)^2), (3*dot_xi*cos(xi))/(cos(xi)^2 - 2), (3*abs(cos(xi)))/(80*(sin(xi)^2 + 1)^(1/2)), xi, i_sav];
% Fxi = [ (150*cos(xi))/(sin(xi)^2 + 1), (150*cos(xi)*sin(xi))/(sin(xi)^2 + 1), angle(((450*sin(xi)^2 - 150)*1i)/(150*(sin(xi)^2 + 1)^2) + (sin(xi)*(sin(xi)^2 - 3))/(sin(xi)^2 + 1)^2), (3*dot_xi*cos(xi))/(cos(xi)^2 - 2), abs(cos(xi))/(50*(sin(xi)^2 + 1)^(1/2)), xi, i_sav];


% ���T�[�W���Ȑ��o�H�C�d�ʑ�
% Fxi = [ 50*cos(5*xi), 50*cos(6*xi), atan2((6*sin(6*xi))/5, -sin(5*xi)), (30*dot_xi*(sin(11*xi) - 11*sin(xi)))/(25*cos(10*xi) + 36*cos(12*xi) - 61), (3*abs((11*sin(xi))/2 - sin(11*xi)/2))/(5*(25*sin(5*xi)^2 + 36*sin(6*xi)^2)^(3/2)), xi, i_sav];


