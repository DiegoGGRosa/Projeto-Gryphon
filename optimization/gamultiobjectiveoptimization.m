% torque optimization - genetic algorithm - PUMA+
% by Diego Rosa - Pontifical Catholic University of Rio de Janeiro

tic % start algorithm time count

% prepare matlab environment
clc
clear all
close all
format short

% declare pitch angle 
alfa = 0; % angle (degree)

% declare vehicle and terrain interaction variables
acm = 4; % maximum linear acceleration (for PUMA+, experimentally obtained)
ms = 0.5; % static friction coefficient
md = 0.3; % dynamical friction coefficient (not used in this code version)
tqmax = 7.21; % maximum mechanical torque after the reduction gears (Nm) -> for PUMA+ -> 7.5 Nm == 52A, so motor == 1.2oz.in/A == 1.31345Nm para 155A == 22.3286Nm na caixa(1:17)

tf = 2; % tempo de simulação
vi = 0; % velocidade inicial
alfa = alfa*pi/180; % angle (rad)
r = 0.1016; % wheel radius (m)
l = 0.31; % lenght - entre eixos (m)
h = 0.1016; % height of the robot GC (m)
m = 19.55; % mass of the robot (Kg)
W = 9.81*m; % weight of the robot (N)
w = W/2; % weight of a half robot (N) 
ep = 0.1*l; % epsilon
bep = atan((l/2-ep)*cos(alfa)/h); % beta epsilon
la = l/2; % 0.5*l*cos(alfa);
lb = la;
% I1 = -2*cos(alfa)*sin(alfa)/(r*l*(cos(alfa)*cos(alfa)-sin(alfa)*sin(alfa)));
% I2 = -2*h*(sin(alfa)*sin(alfa))/(r*l*(cos(alfa)*cos(alfa)-sin(alfa)*sin(alfa)));
% I3 = I1;
% I4 = I2;

%FNB = 0.5*w*cos(alfa)-m*ac*(h/l)-w*sin(alfa)*(h/l)
%FNA = 0.5*w*cos(alfa)+m*ac*(h/l)+w*sin(alfa)*(h/l)

% minimize is the standard optimization in MATLAB:

f = @(TA,TB,ac) [ abs((TA/r)*cos(alfa)+(TB/r)*cos(alfa)-(0.5*w*cos(alfa)+m*ac*(h/l)+w*sin(alfa)*(h/l))*sin(alfa)-(0.5*w*cos(alfa)-m*ac*(h/l)-w*sin(alfa)*(h/l))*sin(alfa)-m*ac*cos(alfa))+abs((TA/r)*sin(alfa)+(TB/r)*sin(alfa)+(0.5*w*cos(alfa)+m*ac*(h/l)+w*sin(alfa)*(h/l))*cos(alfa)+(0.5*w*cos(alfa)-m*ac*(h/l)-w*sin(alfa)*(h/l))*cos(alfa)+m*ac*sin(alfa)-w)+abs((TA/r)*h+(TB/r)*h-(0.5*w*cos(alfa)+m*ac*(h/l)+w*sin(alfa)*(h/l))*la+(0.5*w*cos(alfa)-m*ac*(h/l)-w*sin(alfa)*(h/l))*lb) ; abs(bep-(atan((m*ac*cos(alfa))/(w+m*ac*cos(alfa)*tan(alfa)))))+abs((TA/r)/(0.5*w*cos(alfa)+m*ac*(h/l)+w*sin(alfa)*(h/l))-ms)+abs((TB/r)/(0.5*w*cos(alfa)-m*ac*(h/l)-w*sin(alfa)*(h/l))-ms)]

% obj1:
% -ac
% ou abs(bep-(atan((m*ac*cos(alfa))/(m*ac*cos(alfa)*tan(alfa)))))
% ou soma eq1 eq2 eq3 abs((TA/r)*cos(alfa)+(TB/r)*cos(alfa)-FNA*sin(alfa)-FNB*sin(alfa)-m*acel*cos(alfa))+abs((TA/r)*sin(alfa)+(TB/r)*sin(alfa)+FNA*cos(alfa)+FNB*cos(alfa)+m*acel*sin(alfa)-w)+abs((TA/r)*h+(TB/r)*h-FNA*la+FNB*lb)

% f = @(TA,TB,ac) [ abs((TA/r)/w*cos(alfa)+I1*TA+I2*TB+I3*TA+I4*TB-ms) + abs((TB/r)/w*cos(alfa)+I1*TA+I2*TB+I3*TA+I4*TB-TA/lb-TB/lb-ms)]
fitness = @(ind) f(ind(1),ind(2),ind(3)); % indices = torque A, torque B, normal A, normal B, aceleração

LB = [0; 0; 0];
UB = [tqmax; tqmax; acm]; % stall torque = 0,88 Nm; máxima aceleração = acm
% Aeq = [(cos(alfa)/r)-2*I1*sin(alfa)-2*I3*sin(alfa)+sin(alfa)/lb , (cos(alfa)/r)- 2*I2*sin(alfa)-2*I4*sin(alfa)+sin(alfa)/lb ; (sin(alfa)/r)+2*I1*sin(alfa)+2*I3*sin(alfa)-cos(alfa)/lb , (sin(alfa)/r)+2*I2*sin(alfa)+2*I4*sin(alfa)+cos(alfa)/lb]; 
% beq = [2*w*cos(alfa)*sin(alfa)+m*ac*cos(alfa); -2*w*cos(alfa)*cos(alfa)+w-m*ac*sin(alfa)];

options = gaoptimset('display','off', 'generations', 100000, 'StallGenLimit', 10000000, 'PopulationSize', 1000000);
options = optimoptions('gamultiobj','InitialPopulationRange',[LB';UB']);
options = optimoptions(options,'PlotFcn',{@gaplotpareto});
[x,fval,exitflag,output] = gamultiobj(fitness,3,[],[],[],[],LB,UB,options);
rng default

Ff = x./r;
display(x);
display(fval);

TqA = x(:,1);
TqB = x(:,2);
acel = x(:,3);

FfA = TqA/r;
FfB = TqB/r;

FNB = 0.5*w*cos(alfa)-m*acel*(h/l)-w*sin(alfa)*(h/l);
FNA = 0.5*w*cos(alfa)+m*acel*(h/l)+w*sin(alfa)*(h/l);

dw = ((1-cos(alfa)*cos(alfa))*m*m*acel.^2).^0.5;
ax = acel*cos(alfa);
ay = acel*sin(alfa);

bi = atan(m*acel*cos(alfa))./(w+dw);
bi2 = (1/pi)*180*bi;
bminusb = (1/pi)*180*fval(:,1);
bmb = bminusb;
bep2 = (1/pi)*180*bep;

% Part 2

% f2 = @(TA2,TB2) [abs(bep-atan(m*(-(1/(m*sin(alfa)))*(TA2*(1/r)*sin(alfa)+TB2*(1/r)*sin(alfa)+((w*cos(alfa)-2*cos(alfa)*sin(alfa)*(1/lb)*(TA2*(1/r)+TB2*(1/r)-2*((TA2*(1/r)+TB2*(1/r))*h*(1/lb))*(sin(alfa)*sin(alfa)-cos(alfa)*cos(alfa))))*lb/((la+lb)*(cos(alfa)*cos(alfa)-sin(alfa)*sin(alfa))))*cos(alfa)+(((w*cos(alfa)-2*cos(alfa)*sin(alfa)*(1/lb)*(TA2*(1/r)+TB2*(1/r)-2*((TA2*(1/r)+TB2*(1/r))*h*(1/lb))*(sin(alfa)*sin(alfa)-cos(alfa)*cos(alfa))))*lb/((la+lb)*(cos(alfa)*cos(alfa)-sin(alfa)*sin(alfa))))*la/lb - TA2*(1/r)*h/lb - TB2*(1/r)*h/lb)*cos(alfa)-w))*cos(alfa))./(w+(((1-cos(alfa)*cos(alfa))*m*m*((-(1/(m*sin(alfa)))*(TA2*(1/r)*sin(alfa)+TB2*(1/r)*sin(alfa)+((w*cos(alfa)-2*cos(alfa)*sin(alfa)*(1/lb)*(TA2*(1/r)+TB2*(1/r)-2*((TA2*(1/r)+TB2*(1/r))*h*(1/lb))*(sin(alfa)*sin(alfa)-cos(alfa)*cos(alfa))))*lb/((la+lb)*(cos(alfa)*cos(alfa)-sin(alfa)*sin(alfa))))*cos(alfa)+(((w*cos(alfa)-2*cos(alfa)*sin(alfa)*(1/lb)*(TA2*(1/r)+TB2*(1/r)-2*((TA2*(1/r)+TB2*(1/r))*h*(1/lb))*(sin(alfa)*sin(alfa)-cos(alfa)*cos(alfa))))*lb/((la+lb)*(cos(alfa)*cos(alfa)-sin(alfa)*sin(alfa))))*la/lb - TA2*(1/r)*h/lb - TB2*(1/r)*h/lb)*cos(alfa)-w))).^2).^0.5)))  ,  abs((TA2/r)/((w*cos(alfa)-2*cos(alfa)*sin(alfa)*(1/lb)*(TA2*(1/r)+TB2*(1/r)-2*((TA2*(1/r)+TB2*(1/r))*h*(1/lb))*(sin(alfa)*sin(alfa)-cos(alfa)*cos(alfa))))*lb/((la+lb)*(cos(alfa)*cos(alfa)-sin(alfa)*sin(alfa))))-md)+abs((TB2/r)/(((w*cos(alfa)-2*cos(alfa)*sin(alfa)*(1/lb)*(TA2*(1/r)+TB2*(1/r)-2*((TA2*(1/r)+TB2*(1/r))*h*(1/lb))*(sin(alfa)*sin(alfa)-cos(alfa)*cos(alfa))))*lb/((la+lb)*(cos(alfa)*cos(alfa)-sin(alfa)*sin(alfa))))*la/lb - TA2*(1/r)*h/lb - TB2*(1/r)*h/lb)-md)  ];
% 
% fitness2 = @(ind2) f2(ind2(1),ind2(2)); % indices 1,2 = torques TA,TB

% LB = [0; 0];
% UB = [5*0.88; 5*0.88]; % stall torque = 0,88 Nm
% options = gaoptimset('display','off', 'generations', 10000, 'StallGenLimit', 100000, 'PopulationSize', 1000); % 'off', 'iter'
% [x2, fval2] = gamultiobj(fitness2, 2, [], [], [], [], LB, UB, [], options);
% 
% display(x2);
% display(fval2);

% TqA2 = x2(:,1);
% TqB2 = x2(:,2);
% FfA2 = TqA2/r;
% FfB2 = TqB2/r;
% FNA2 = (w*cos(alfa)-2*cos(alfa)*sin(alfa)*(1/lb)*(TqA2*(1/r)+TqB2*(1/r)-2*((TqA2*(1/r)+TqB2*(1/r))*h*(1/lb))*(sin(alfa)*sin(alfa)-cos(alfa)*cos(alfa))))*lb/((la+lb)*(cos(alfa)*cos(alfa)-sin(alfa)*sin(alfa)));
% FNB2 = FNA2*la/lb - TqA2*(1/r)*h/lb - TqB2*(1/r)*h/lb;
% a2 = (-(1/(m*sin(alfa)))*(TqA2*(1/r)*sin(alfa)+TqB2*(1/r)*sin(alfa)+FNA2*cos(alfa)+FNB2*cos(alfa)-w));
% ax2 = a2*cos(alfa);
% ay2 = a2*sin(alfa);

Pmax = 12; % maximum power (P = EI) [W]
% obs.: this maximum power can reaches 80 W for a few instants

[ac,aci] = max(acel);
Ia = 5.68*TqA(aci);
Ib = 5.68*TqB(aci);

% passo 1 = máxima aceleração sem perder estabilidade
% passo 2 = reduzir torque para aumentar tensão (potência limitada)
% [ac2,aci2] = min(a2);
% Ia2 = 5.68*TqA2(aci2);
% Ib2 = 5.68*TqB2(aci2);

Ea = Pmax/(max(Ia,Ib));
Eb = Ea;

vmax = 1.76; % maximum velocity [m/s] (4rps == 16V)
vf = 2*pi*r*0.25*Ea; % valor em m/s (1V == 0,25 rps)
t = vf/ac;

% k = 0:0.01:t;
% P = t:0.01:tf;
% E = (Ea/t)*k;
% figure(5);
% K = [k P];
% Em = [E Ea*ones(size(P))];
% plot(K,Em);
% xlabel('Time (s)','FontSize',12);
% title('Electrial tension in motors A and B','FontSize',12);
% ylabel('Tension (V)','FontSize',12);
% grid;
% 
% V = 2*pi*r*0.25*E;
% figure(2);
% p = [V vf*ones(size(P))];
% plot(K,p);
% xlabel('Time (s)','FontSize',12);
% title('Robot velocity','FontSize',12);
% ylabel('Velocity (m/s)','FontSize',12);
% grid;
% 
% figure(3);
% title('Torques no motor A - rascunho','FontSize',12);
% T1 = TqA(aci);
% T2 = TqA2(aci2);
% R = [k P];
% T = [T1*ones(size(k)) T2*ones(size(P))];
% plot(R,T);
% grid;
% 
% figure(4);
% title('Torques no motor B - rascunho','FontSize',12);
% T12 = TqB(aci);
% T22 = TqB2(aci2);
% TQ = [T12*ones(size(k)) T22*ones(size(P))];
% plot(R,TQ);
% grid;
% 
% Ilow = T2*5.68;
% Elow = Pmax/Ilow;
% Vlow = 2*pi*r*0.25*Elow
% % obs.: restriction related to wheel A (B is far of Pmax limitation)

if  Ia>50 || Ib>50
    display ('This movement is not possible to be achieved');    
elseif ac*FNB(aci)<0
       display ('This movement is going to be unstable');
elseif FNB(aci)<0
       display ('This movement is unstable');
elseif ac<0
       display ('It is not possible to accomplish the trajectory');
end

% x indica os torques nas rodas traseiras e dianteiras
% fval indica, na mesma sequência de x, os valores na pareto front

soma_EQ1 = FfA*cos(alfa)+FfB*cos(alfa)-FNA*sin(alfa)-FNB*sin(alfa)-m*acel*cos(alfa);
soma_EQ2 = FfA*sin(alfa)+FfB*sin(alfa)+FNA*cos(alfa)+FNB*cos(alfa)+m*acel*sin(alfa)-w;
soma_EQ3 = FfA*h+FfB*h-FNA*la+FNB*lb;

soma_abs = abs(soma_EQ1)+abs(soma_EQ2)+abs(soma_EQ3);

coefB = FfB./FNB
coefA = FfA./FNA

Ttras_T_diant_acel = x
e_obj1equilibroio_e_obj2estabilidade = fval

toc % finish algorithm time count
