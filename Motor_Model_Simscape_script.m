%% DC MOTOR PARAMETER ESTIMATION USING LEAST SQAURE AND SEARCH SPACE
% This code will help you to indentify parametrs of DC motors for better
% control. Idea is to have measurement of Voltage, Current and Motor
% Position to estimate motor parameters.

% Author: Shail Jadav
% Visit:  shailjadav.github.io
% June 5,2021
%% Intialise
clear
close all
clc
%% Sorting of the data
Data=xlsread('MotorDATA.xlsx');
R=2.2; %measured
c1=1;
F1Data=[0,0,0,0];
for i=1:1:length(Data)
    if(isnan(Data(i,3))==0)
        F1Data(c1,:)=Data(i,:);
        c1=c1+1;
    end
end
F1Data=F1Data(21:end,:); % initial garbage removal
F1Data(:,1)=(F1Data(:,1)-F1Data(1,1));
F1Data(:,3)=(F1Data(:,3)-F1Data(1,3));
F1Data(:,4)=(F1Data(:,4)-F1Data(1,4));
F1Data(:,2)=F1Data(:,2).*24;
F1Data(1,2)=0;
%% Measurement with Noise
ws=2; %window size
t=F1Data(:,1);
V=smooth(F1Data(:,2),ws); 
I=smooth(F1Data(:,3),ws);
dI=smooth(diff(I)./diff(t),ws);
TH=smooth(F1Data(:,4),ws);
dTH=smooth(diff(TH)./diff(t),ws);
ddTH=smooth(diff(dTH)./diff(t(1:end-1,:)),ws);
tout=t;
dtout=t(1:end-1,:);
ddtout=t(1:end-2,:);

figure
subplot(231)
plot(tout,TH)
hold on
subplot(232)
plot(dtout,dTH)
hold on
subplot(233)
plot(ddtout,ddTH)
hold on
subplot(234)
plot(tout,I)
hold on
subplot(235)
plot(dtout,dI)
hold on
subplot(236)
plot(tout,V)
hold on

%% Find L R K
clear Data
% Inital Least squares
for i=1:1:floor(length(I))-1
    Data=[dI(i); I(i); dTH(i)];
    IP=V(i);
    MAP(i,:)=IP*pinv(Data);
end

f=1;
for i=1:1:length(MAP)
    if(MAP(i,2)>R-0.5 && MAP(i,2)< R+0.5 )
        if(MAP(i,1)>0)
            if(MAP(i,3)>0)
                MAPF(f,:)=MAP(i,:);
                f=f+1;
            end
        end
    end
end

Re=sum(MAPF(:,2))/size(MAPF,1);
Le=sum(MAPF(:,1))/size(MAPF,1);
Ke=sum(MAPF(:,3))/size(MAPF,1);
%% First Filtering

FG=[Le Re Ke];

L1=Le - (Le/2);

R1=Re - (Re/2);

K1=Ke - (Ke/2);

cnt=1;
L11=L1;
R11=R1;
K11=K1;
I=I(1:end-1,1);
for i=1:1:20
    L11=L11+(Le/(20));
    R11=R1;
    for j=1:1:20
        R11=R11+(Re/(20));
        K11=K1;
        for k=1:1:20
            K11=K11+(Ke/(20));
            
            Data=[dI  I  dTH];
            Ve=Data*[L11 R11 K11]';
            E(cnt,1)=norm(V(1:end-1,1)-Ve);
            E(cnt,2)=L11;
            E(cnt,3)=R11;
            E(cnt,4)=K11;
            cnt=cnt+1;
        end
    end
end

for i=1:1:length(E)
    if(min(E(:,1))==E(i,1))
        FDATA=E(i,:);
    end
end

%% Second Filtering
FG=FDATA(1,2:end);
Le=FDATA(1,2);
Re=FDATA(1,3);
Ke=FDATA(1,4);

L1=Le - (Le/2);

R1=Re - (Re/2);

K1=Ke - (Ke/2);

cnt=1;
L11=L1;
R11=R1;
K11=K1;

for i=1:1:10
    L11=L11+(Le/(10));
    R11=R1;
    for j=1:1:10
        R11=R11+(Re/(10));
        K11=K1;
        for k=1:1:10
            K11=K11+(Ke/(10));
            
            Data=[dI  I  dTH];
            Ve=Data*[L11 R11 K11]';
            E(cnt,1)=norm(V(1:end-1,1)-Ve);
            E(cnt,2)=L11;
            E(cnt,3)=R11;
            E(cnt,4)=K11;
            cnt=cnt+1;
        end
    end
end

for i=1:1:length(E)
    if(min(E(:,1))==E(i,1))
        SDATA=E(i,:);
    end
end

clearvars -except SDATA FDATA V I dI ddTH dTH TH tout
%% Find J b
% Inital Least squares

for i=1:1:floor(length(I))-1
    Data=[ddTH(i); dTH(i)];
    IP=SDATA(1,4)*I(i);
    MAP(i,:)=IP*pinv(Data);
end

f=1;
for i=1:1:length(MAP)
    if(MAP(i,1)>0)
        if(MAP(i,2)>0)
            MAPF(f,:)=MAP(i,:);
            f=f+1;
        end
    end
end


Je=sum(MAPF(:,1))/size(MAPF,1);
be=sum(MAPF(:,2))/size(MAPF,1);


%% First Filtering
FG=[Je be];

J1=Je - (Je/2);
b1=be - (be/2);


cnt=1;
J11=J1;
b11=b1;

dTH=dTH(1:end-1,1);
I=I(1:end-1,1);
for i=1:1:20
    J11=J11+(Je/(20));
    b11=b1;
    for j=1:1:20
        b11=b11+(be/(20));
        
        
        Data=[ddTH  dTH];
        Ve=Data*[J11 b11]';
        E(cnt,1)=norm(SDATA(1,4)*I - Ve);
        E(cnt,2)=J11;
        E(cnt,3)=b11;
        cnt=cnt+1;
    end
end


for i=1:1:length(E)
    if(min(E(:,1))==E(i,1))
        F1DATA=E(i,:);
    end
end

%% Second Filtering

FG=F1DATA(1,2:end);

Je=F1DATA(1,2);
be=F1DATA(1,3);

cnt=1;
J11=J1;
b11=b1;

for i=1:1:10
    J11=J11+(Je/(10));
    b11=b1;
    for j=1:1:10
        b11=b11+(be/(10));
        
        
        Data=[ddTH  dTH];
        Ve=Data*[J11 b11]';
        E(cnt,1)=norm(SDATA(1,4)*I - Ve);
        E(cnt,2)=J11;
        E(cnt,3)=b11;
        cnt=cnt+1;
    end
end


for i=1:1:length(E)
    if(min(E(:,1))==E(i,1))
        S1DATA=E(i,:);
    end
end

%% Display
clc
disp(strcat("Estimated L","=",num2str(SDATA(1,2))))
disp(strcat("Estimated R","=",num2str(SDATA(1,3))))
disp(strcat("Estimated K","=",num2str(SDATA(1,4))))
disp(strcat("Estimated J","=",num2str(S1DATA(1,2))))
disp(strcat("Estimated b","=",num2str(S1DATA(1,3))))


% Validation
L=(SDATA(1,2));
R=(SDATA(1,3));
K=(SDATA(1,4));
J=S1DATA(1,2);
b=S1DATA(1,3);

VIP=[tout V];

%sim('Motor_Model_Simscape_17b.mdl'); %if you need older version
sim('Motor_Model_Simscape.mdl');

V=Voltage.signals.values ;
I=Current.signals.values ;
dI=CurrentDerivative.signals.values ;
dTH=Velocity.signals.values ;
ddTH=Acc.signals.values ;
TH=Position.signals.values;


subplot(231)
plot(tout,TH)
grid minor
xlabel('Time(s)','interpreter','latex','fontsize',14)
ylabel('Angular position(rad)','interpreter','latex','fontsize',14)
xlim([0 100])

subplot(232)
plot(tout,dTH)
grid minor
xlabel('Time(s)','interpreter','latex','fontsize',14)
ylabel('Angular velocity(rad /s)','interpreter','latex','fontsize',14)
xlim([0 100])

subplot(233)
plot(tout,ddTH)
grid minor
xlabel('Time(s)','interpreter','latex','fontsize',14)
ylabel('Angular acceleration(rad /s*s)','interpreter','latex','fontsize',14)
xlim([0 100])

subplot(234)
plot(tout,I)
grid minor
xlabel('Time(s)','interpreter','latex','fontsize',14)
ylabel('Current (A)','interpreter','latex','fontsize',14)
xlim([0 100])

subplot(235)
plot(tout,dI)
grid minor
xlabel('Time(s)','interpreter','latex','fontsize',14)
ylabel('Current Derivative(A/s)','interpreter','latex','fontsize',14)
xlim([0 100])

subplot(236)
plot(tout,V)
grid minor
xlabel('Time(s)','interpreter','latex','fontsize',14)
ylabel('Voltage(V)','interpreter','latex','fontsize',14)
xlim([0 100])



