clc
close all
clear

%% �V�~�����[�V��������
dt = 0.001; 
t_end =50;
time =0:dt:t_end;

%%�ߎ�

r_kinzi = 0; %r=\dot{chi}�̋ߎ����s�����D�@0:�s��Ȃ��@1:�s��

%% �e�p�����[�^
%�G���x�[�^����Ɋւ���p�����[�^
a_ue =60;
b_ue = 60;

fp_ue=1.3; %P�Q�C��
fd_ue=0.045; %D�Q�C��

%�G����������Ɋւ���p�����[�^
a_ua=40;
b_ua=0.8;

fp_ua=1.5; %P�Q�C��
fd_ua=0.31; %D�Q�C��

%�X���b�g������Ɋւ���p�����[�^
Vr = 12; %�ڕW�΋C���x
kp_Th = 20; %P�Q�C��
kd_Th = 0.1; %D�Q�C��

%���O���蓖��(�v�Z�������̂��߁C���炩���ߕϐ����i�[���锠������Ă��邾��)

dot_u = zeros([length(time),1]);
dot_v = zeros([length(time),1]);
dot_w = zeros([length(time),1]);

dot_p = zeros([length(time),1]);
dot_q = zeros([length(time),1]);
dot_r = zeros([length(time),1]);

dot_phi = zeros([length(time),1]);
dot_theta= zeros([length(time),1]);
dot_psi = zeros([length(time),1]);

dot_X = zeros([length(time),1]);
dot_Y = zeros([length(time),1]);
dot_Z = zeros([length(time),1]);


u= zeros([length(time),1]);
v= zeros([length(time),1]);
w= zeros([length(time),1]);

p= zeros([length(time),1]);
q= zeros([length(time),1]);
r= zeros([length(time),1]);

phi= zeros([length(time),1]);
theta= zeros([length(time),1]);
psi= zeros([length(time),1]);

X= zeros([length(time),1]);
Y= zeros([length(time),1]);
Z= zeros([length(time),1]);


u_a = zeros([length(time),1]);
u_e = zeros([length(time),1]);
Ft = zeros([length(time),1]);

chi= zeros([length(time),1]);

Va= zeros([length(time),1]);
alpha= zeros([length(time),1]);
beta = zeros([length(time),1]);

Vg=zeros([length(time),1]);


phi_r  = zeros([length(time),1]);
dphi_r = zeros([length(time),1]);

theta_r  = zeros([length(time),1]);
dtheta_r = zeros([length(time),1]);

% 
gamma= zeros([length(time),1]);
dgamma= zeros([length(time),1]);


%% �萔�ݒ�

g = 9.80665; %�d�͉����x
W = 2.0; %����
m = 2.320; %�@�̏d��
Ixx = 0.1580; %x���������[�����g
Iyy = 0.07152; %y���������[�����g
Izz = 0.1959; %z���������[�����g
S = 0.42; %���ʐ�
c = 0.32; %������
rho = 1.155; %��C���x

%���t�_
alpha_0 = 0.0905;
elev_0 = 14.5311;
Ft_0 = 3.1701;

% �����l�ݒ�

u(1,1)=11;
v(1,1)=0;
w(1,1)=0;

p(1,1)=0;
q(1,1)=0;
r(1,1)=0;

phi(1,1)=0;
theta(1,1)=0;
psi(1,1)=0;

X(1,1)=0;
Y(1,1)=5;
Z(1,1)=5;

%% �V�~�����[�V����

for i = 1 : length(time)
    
    %���Ԃ�\��
    t = i*dt

%% �ϐ��ݒ�



Va(i,1) =sqrt(u(i,1)^2+v(i,1)^2+w(i,1)^2);
Vg(i,1)=Va(i,1); 

alpha(i,1) = atan(w(i,1)/u(i,1));

if alpha(i,1) >0.1745
    alpha(i,1) =0.1745;
elseif alpha(i,1) < -0.3142
    alpha(i,1) = -0.3142;
end
beta(i,1) = atan(v(i,1)/Va(i,1));

gamma(i,1) = theta(i,1)-alpha(i,1);
chi(i,1) = psi(i,1)+beta(i,1);


%% �G���x�[�^�������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


 
    
 if i == 1
     dgamma(i,1) = 0;
 else
     dgamma(i,1) = (gamma(i,1) - gamma(i - 1,1)) / dt;
 end 

 if dgamma(i,1) >= 100 * pi / 180
     dgamma(i,1) = 100 * pi / 180;
 elseif dgamma(i,1) <= - 100 * pi / 180
     dgamma(i,1) = - 100 * pi / 180;
 end

 %�ڕW�s�b�`�p�̌v�Z
    theta_r(i,1) =  -( ( Vg(i,1) * sin(gamma(i,1)) + a_ue*dgamma(i,1) ) / b_ue + Z(i,1)) / a_ue+alpha(i,1);

    % �ڕW�s�b�`�p�̏�������l��ݒ�
    if theta_r(i,1) >= 30 * pi / 180
        theta_r(i,1) = 30 * pi / 180;
    elseif theta_r(i,1) <= - 30 * pi / 180
        theta_r(i,1) = - 30 * pi / 180;
    end
      
    %�ڕW�s�b�`�p���x�̌v�Z
    if i == 1
        dtheta_r(i,1) = 0;
    else
        dtheta_r(i,1) = (theta_r(i,1) - theta_r(i - 1,1)) / dt;
    end 

        if dtheta_r(i,1) >= 60 * pi / 180
        dtheta_r(i,1) = 60 * pi / 180;
    elseif dtheta_r(i,1) <= - 60 * pi / 180
        dtheta_r(i,1) = - 60 * pi / 180;
        end


    %�G���x�[�^���͂̌v�Z
      u_e(i,1) = (- fp_ue * (theta(i,1) - theta_r(i,1)) - fd_ue * (q(i,1) - dtheta_r(i,1)) )*180/pi+elev_0;
 %�G���x�[�^���͂̌v�Z
 %u_e(i,1) = -(Vg(i,1) * sin(gamma(i,1))+b_ue*( Z(i,1)+a_ue*dgamma(i,1)))/(a_ue*K_ue)+elev_0;
 %u_e(i,1) =0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% �G�������������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


phi_r(i,1) = atan((Vg(i,1)/(a_ua*g))*(-b_ua*(Y(i,1)+a_ua*chi(i,1))-Vg(i,1)*sin(chi(i,1))));

    % �ڕW�s�b�`�p�̏�������l��ݒ�
    if phi_r(i,1) >= 60 * pi / 180
        phi_r(i,1) = 60 * pi / 180;
    elseif phi_r(i,1) <= - 60 * pi / 180
        phi_r(i,1) = - 60 * pi / 180;
    end

if i == 1
    dphi_r(i,1) = 0;
else
    dphi_r(i,1) = (phi_r(i,1)-phi_r(i-1,1))/dt;
end 

    if dphi_r(i,1) >= 60 * pi / 180
        dphi_r(i,1) = 60 * pi / 180;
    elseif dphi_r(i,1) <= - 60 * pi / 180
        dphi_r(i,1) = - 60 * pi / 180;
    end

 %�G���������͂̌v�Z
u_a(i,1) = (-fp_ua*(phi(i,1)-phi_r(i,1))-fd_ua*(p(i,1) - dphi_r(i,1)))*180/pi;


%u_a(i,1) = (-fp_ua*Y(i,1)-fd_ua*dot_Y(i,1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% �~�L�V���O%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% �G���x�[�^���͂�-20deg~20deg�ɐ���
if u_e(i,1) > 20
    u_e(i,1) = 20;
elseif u_e(i,1) < -20
    u_e(i,1) = -20;
end

% �G���������͂�-20deg~20deg�ɐ���
if u_a(i,1) > 20
    u_a(i,1) = 20;
elseif u_a(i,1) < -20
    u_a(i,1) = -20;
end

%�~�L�V���O
d_R = u_e(i,1) + u_a(i,1);
d_L = u_e(i,1) - u_a(i,1);

% �E���G���{���Ǌp��-20deg~20deg�ɐ���
if d_R > 20
    d_R = 20;
elseif d_R < -20
    d_R = -20;
end

% �����G���{���Ǌp��-20deg~20deg�ɐ���
if d_L > 20
    d_L = 20;
elseif d_L < -20
    d_L = -20;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% �X���b�g���������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if i==1
    Ft(i,1) = -kp_Th*(Va(i,1) - Vr)-kd_Th*0 + Ft_0;
else 
    Ft(i,1) = -kp_Th*(Va(i,1) - Vr)-kd_Th*( (Va(i,1)-Va(i-1,1))/dt ) + Ft_0;
end

if Ft(i,1) > 2*Ft_0
    Ft(i,1) = 2*Ft_0;
elseif Ft(i,1) < 0
    Ft(i,1) = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% �V�~�����[�V�����̎�

C_L_mat = [-1.666*1e-7 -1.272*1e-6 1.034*1e-5 -5.228*1e-5 0.04190 0.2232 -3.358*1e-5 -0.007591];
C_D_mat = [6.713*1e-8 5.151*1e-8 -3.298*1e-5 5.992*1e-4 0.006312 0.03464 3.133*1e-5 -0.002794];

C_Mx_mat =[5.748*1e-8 6.026*1e-7 -5.592*1e-6 -3.944*1e-5 -0.01022 -0.05403 1.263*1e-5 0.00225];
C_My_mat =[2.064*1e-8 1.247*1e-6 6.259*1e-6 -3.454*1e-4 -0.007662 -0.03874 7.805*1e-6 0.005799];
C_Mz_mat =[2.092*1e-8 5.792*1e-7 -4.378*1e-6 -1.885*1e-4 -1.145*1e-4 0.006961 9.491*1e-6 -8.274*1e-5];

d_R_mat = [(alpha(i,1)*180/pi)^5 ; (alpha(i,1)*180/pi)^4 ; (alpha(i,1)*180/pi)^3 ; (alpha(i,1)*180/pi)^2 ; (alpha(i,1)*180/pi);1 ; d_R^2;d_R];
d_L_mat = [(alpha(i,1)*180/pi)^5 ; (alpha(i,1)*180/pi)^4 ; (alpha(i,1)*180/pi)^3 ; (alpha(i,1)*180/pi)^2 ; (alpha(i,1)*180/pi);1 ; d_L^2;d_L];

C_L = C_L_mat*(d_R_mat+d_L_mat);
C_D = C_D_mat*(d_R_mat+d_L_mat);

C_Mx= C_Mx_mat*(d_R_mat - d_L_mat);
C_My = C_My_mat*(d_R_mat + d_L_mat);
C_Mz= C_Mz_mat*(d_R_mat - d_L_mat);

L = 0.5*rho*S*Va(i,1)^2*C_L;
D = 0.5*rho*S*Va(i,1)^2*C_D;

Mx= 0.5*rho*S*W*Va(i,1)^2*C_Mx;
My= 0.5*rho*S*c*Va(i,1)^2*C_My;
Mz= 0.5*rho*S*W*Va(i,1)^2*C_Mz;


%% ���f����%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dot_u(i,1) = -q(i,1)*w(i,1)+r(i,1)*v(i,1)+(Ft(i,1)-m*g*sin(theta(i,1))+L*sin(alpha(i,1))-D*cos(alpha(i,1))*cos(beta(i,1)) )/m;
dot_v(i,1) = -r(i,1)*u(i,1)+p(i,1)*w(i,1)+(m*g*sin(phi(i,1))*cos(theta(i,1))-D*sin(beta(i,1)) )/m;
dot_w(i,1) = -p(i,1)*v(i,1)+q(i,1)*u(i,1)+(m*g*cos(phi(i,1))*cos(theta(i,1))-L*cos(alpha(i,1))-D*sin(alpha(i,1))*cos(beta(i,1)) )/m;

dot_p(i,1) = (Iyy-Izz)/Ixx*q(i,1)*r(i,1)+Mx/Ixx; %p
dot_q(i,1) = (Izz-Ixx)/Iyy*r(i,1)*p(i,1)+My/Iyy; %q
dot_r(i,1) = (Ixx-Iyy)/Izz*p(i,1)*q(i,1)+Mz/Izz; %r

dot_phi(i,1) = p(i,1)+q(i,1)*sin(phi(i,1))*tan(theta(i,1))+r(i,1)*cos(phi(i,1))*tan(theta(i,1)); %phi
dot_theta(i,1) = q(i,1)*cos(phi(i,1))-r(i,1)*sin(phi(i,1)); %theta

if r_kinzi ==0
    dot_psi(i,1) = q(i,1)*sin(phi(i,1))/cos(theta(i,1))+r(i,1)*cos(phi(i,1))/cos(theta(i,1));
else
    dot_psi(i,1) =  g/(Vg(i,1)*cos(gamma(i,1)))*tan(phi(i,1));
end

dot_X(i,1) = u(i,1)*cos(psi(i,1))*cos(theta(i,1))+v(i,1)*(cos(psi(i,1))*sin(theta(i,1))*sin(phi(i,1))-sin(psi(i,1))*cos(phi(i,1)))+w(i,1)*(cos(psi(i,1))*sin(theta(i,1))*cos(phi(i,1))+sin(psi(i,1))*sin(phi(i,1))); %X
dot_Y(i,1) = u(i,1)*sin(psi(i,1))*cos(theta(i,1))+v(i,1)*(sin(psi(i,1))*sin(theta(i,1))*sin(phi(i,1))+cos(psi(i,1))*cos(phi(i,1)))+w(i,1)*(sin(psi(i,1))*sin(theta(i,1))*cos(phi(i,1))-cos(psi(i,1))*sin(phi(i,1)));; %Y
dot_Z(i,1) = u(i,1)*sin(theta(i,1))-v(i,1)*cos(theta(i,1))*sin(phi(i,1))-w(i,1)*cos(theta(i,1))*cos(phi(i,1)); %Z

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ���̃X�e�b�v�̏�Ԃ̌v�Z�i�P���ϕ��j%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u(i+1,1)= u(i,1) + dot_u(i,1)*dt;
v(i+1,1)= v(i,1) + dot_v(i,1)*dt;
w(i+1,1)= w(i,1) + dot_w(i,1)*dt;

p(i+1,1)= p(i,1) + dot_p(i,1)*dt;
q(i+1,1)= q(i,1) + dot_q(i,1)*dt;
r(i+1,1)= r(i,1) + dot_r(i,1)*dt;

phi(i+1,1)= phi(i,1) + dot_phi(i,1)*dt;
theta(i+1,1)= theta(i,1) + dot_theta(i,1)*dt;
psi(i+1,1) = psi(i,1) + dot_psi(i,1)*dt;

X(i+1,1)= X(i,1) + dot_X(i,1)*dt;
Y(i+1,1)= Y(i,1) + dot_Y(i,1)*dt;
Z(i+1,1)= Z(i,1) + dot_Z(i,1)*dt;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%alpha(i+1,1)= alpha(i,1) + dot_alpha(i,1)*dt;
%psi(i+1,1)= psi(i,1) + dot_psi(i,1)*dt;

end 
%% �V�~�����[�V�����I��
u(end,:) = [];
v(end,:) = [];
w(end,:) = [];

p(end,:) = [];
q(end,:) = [];
r(end,:) = [];
X(end,:) = [];
Y(end,:) = [];
Z(end,:) = [];
phi(end,:) = [];
theta(end,:) = [];
alpha(end,:) = [];
psi(end,:) = [];

%phi_r(end,:) = [];
%% ���ʃv���b�g

%�G���x�[�^����
figure('Name','u_e','NumberTitle','off');
plot(time,u_e,'b-','LineWidth',2);
yline(elev_0,'r--','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$\delta_e$[deg.]','Interpreter','latex','FontSize',15);

%�G����������
figure('Name','u_a','NumberTitle','off');
plot(time,u_a,'b-','LineWidth',2);
yline(0,'r--','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$\delta_a$[deg.]','Interpreter','latex','FontSize',15);

%�X���b�g������
figure('Name','Ft','NumberTitle','off');
plot(time,Ft,'b-','LineWidth',2);
yline(Ft_0,'r--','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$Ft$[N]','Interpreter','latex','FontSize',15);


%���i���x
%Va,u,v,w
figure('Name','Va','NumberTitle','off');
plot(time,Va,'b-','LineWidth',2);
yline(12,'r--','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$Va$[m/s]','Interpreter','latex','FontSize',15);

figure('Name','u','NumberTitle','off');
plot(time,u,'b-','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$u$[m/s]','Interpreter','latex','FontSize',15);

figure('Name','v','NumberTitle','off');
plot(time,v,'b-','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$v$[m/s]','Interpreter','latex','FontSize',15);

figure('Name','w','NumberTitle','off');
plot(time,w,'b-','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$w$[m/s]','Interpreter','latex','FontSize',15);

%�p���x
%p,q,r
figure('Name','p','NumberTitle','off');
plot(time,p*180/pi,'b-','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$p$[deg./s]','Interpreter','latex','FontSize',15);

figure('Name','q','NumberTitle','off');
plot(time,q*180/pi,'b-','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$q$[deg./s]','Interpreter','latex','FontSize',15);

figure('Name','r','NumberTitle','off');
plot(time,r*180/pi,'b-','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$r$[deg./s]','Interpreter','latex','FontSize',15);

%�p�x
%phi,theta,psi
figure('Name','phi','NumberTitle','off');
plot(time,phi*180/pi,'b-','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$\phi$[deg.]','Interpreter','latex','FontSize',15);

figure('Name','theta','NumberTitle','off');
plot(time,theta*180/pi,'b-','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$\theta$[deg.]','Interpreter','latex','FontSize',15);

figure('Name','psi','NumberTitle','off');
plot(time,psi*180/pi,'b-','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$\psi$[deg.]','Interpreter','latex','FontSize',15);

%x,y,z�΍�
figure('Name','X','NumberTitle','off');
plot(time,X,'b-','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$x$[m]','Interpreter','latex','FontSize',15);

figure('Name','Y','NumberTitle','off');
plot(time,Y,'b-','LineWidth',2);
yline(0,'r--','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$y$[m]','Interpreter','latex','FontSize',15);

figure('Name','Z','NumberTitle','off');
plot(time,Z,'b-','LineWidth',2);
yline(0,'r--','LineWidth',2);
xlim([time(1,1) time(1,end)]);
xlabel('t[s]','FontSize',15);
ylabel('$z$[m]','Interpreter','latex','FontSize',15);




% figure('Name','chi','NumberTitle','off');
% plot(time,chi*180/pi,'b-','LineWidth',2);
% xlim([time(1,1) time(1,end)]);
% xlabel('t[s]','FontSize',15);
% ylabel('$\chi$[deg.]','Interpreter','latex','FontSize',15);
% 
% figure('Name','*phi_r','NumberTitle','off');
% subplot(2,1,1)
% hold on
% plot(time,phi*180/pi,'b-','LineWidth',1.5);
% plot(time,phi_r*180/pi,'r--','LineWidth',1.5);
% xlim([time(1,1) time(1,end)]);
% ylabel('$$\phi,\phi_r$$[deg.]','Interpreter','latex','FontSize',10);
% legend('���[���p','�ڕW���[���p','FontSize',10,'Location','best')
% hold off
% subplot(2,1,2)
% hold on
% plot(time,p*180/pi,'b-','LineWidth',1.5);
% plot(time,dphi_r*180/pi,'r--','LineWidth',1.5);
% xlim([time(1,1) time(1,end)]);
% xlabel('t[s]','FontSize',15);
% ylabel('$$\dot{\phi},\dot{\phi}_r$$[deg./s]','Interpreter','latex','FontSize',10);
% legend('���[���p���x','�ڕW���[���p���x','FontSize',10,'Location','best')
% hold off
%     
% %�s�b�`�p�ƃs�b�`�p���x
% figure('Name','*theta_r','NumberTitle','off');
% subplot(2,1,1)
% hold on
% plot(time,theta*180/pi,'b-','LineWidth',1.5);
% plot(time,theta_r*180/pi,'r--','LineWidth',1.5);
% xlim([time(1,1) time(1,end)]);
% ylabel('$$\theta,\theta_r$$[deg.]','Interpreter','latex','FontSize',10);
% legend('�s�b�`�p','�ڕW�s�b�`�p','FontSize',10,'Location','best')
% hold off
% subplot(2,1,2)
% hold on
% plot(time,q*180/pi,'b-','LineWidth',1.5);
% plot(time,dtheta_r*180/pi,'r--','LineWidth',1.5);
% xlim([time(1,1) time(1,end)]);
% xlabel('t[s]','FontSize',15);
% ylabel('$$\dot{\theta},\dot{\theta}_r$$[deg./s]','Interpreter','latex','FontSize',10);
% legend('�s�b�`�p���x','�ڕW�s�b�`�p���x','FontSize',10,'Location','best')
% hold off