%% 三轴平台式惯导数字样机入口
clear; clc; close all;
p=par;
rng(p.seed,'twister');
assignin('base','p',p);

if ~exist('model.slx','file'), make; end

%% 1. 构造第一版力矩扰动输入
t=(0:p.ts:p.stop).';
m=zeros(numel(t),3);
for i=1:3
    a=p.ax(i);
    m(:,i)=0.22*a.distMax*sin(2*pi*(0.30+0.08*i)*t) ...
        +0.10*a.distMax*sin(2*pi*(1.2+0.2*i)*t+0.4*i) ...
        +a.imbMax*sin(2*pi*0.05*t+0.7*i);
end
m(t>=15 & t<15.4,1)=m(t>=15 & t<15.4,1)+0.7*p.ax(1).distMax;
m(t>=30 & t<30.3,2)=m(t>=30 & t<30.3,2)-0.7*p.ax(2).distMax;
m(t>=45 & t<45.2,3)=m(t>=45 & t<45.2,3)+0.8*p.ax(3).distMax;

mx=timeseries(m(:,1),t); my=timeseries(m(:,2),t); mz=timeseries(m(:,3),t);
assignin('base','mx',mx); assignin('base','my',my); assignin('base','mz',mz);

%% 2. 导航层与三轴稳定回路协同闭合
% 第一次从零指令开始；导航解算得到游移方位指令后，再送回稳定回路。
cmd=zeros(numel(t),3);
for it=1:p.loop.iter
    cx=timeseries(cmd(:,1),t); cy=timeseries(cmd(:,2),t); cz=timeseries(cmd(:,3),t);
    assignin('base','cx',cx); assignin('base','cy',cy); assignin('base','cz',cz);

    simOut=sim('model','StopTime',num2str(p.stop));
    [out,d,ang,rate,cur,tn,cpTrue]=decodeNav(simOut,p);
    cmdNav=navCommand(out,ang,p);

    if it<p.loop.iter
        cmdNew=interp1(tn,cmdNav,t,'linear','extrap');
        cmd=(1-p.loop.relax)*cmd+p.loop.relax*cmdNew;
    end
    fprintf('导航-稳定回路协同迭代：%d/%d\n',it,p.loop.iter);
end
cmdApplied=cmd;

angTs=simOut.get('ang'); rateTs=simOut.get('rate');
curTs=simOut.get('cur'); torTs=simOut.get('tor');
satTs=simOut.get('sat'); rawTs=simOut.get('raw');

%% 4. 结果整理
r.t=angTs.Time;
r.ang=angTs.Data; r.rate=rateTs.Data;
r.cur=curTs.Data; r.tor=torTs.Data;
r.sat=logical(satTs.Data); r.raw=rawTs.Data;
r.nav=out; r.input=d; r.par=p;
r.truth.cpn=cpTrue;
r.cmd=cmdApplied;
r.cmdNav=cmdNav;
r.satRatio=mean(r.sat,1);
r.maxRate=rad2deg(max(abs(r.rate),[],1));
r.maxCur=max(abs(r.cur),[],1);
r.maxTor=max(abs(r.tor),[],1);

fprintf('\n========== 三轴平台式惯导数字样机 ==========' );
fprintf('\n仿真时长：%.1f s，稳定回路：%.0f Hz，导航：%.0f Hz\n', ...
    p.stop,1/p.ts,1/p.tn);
fprintf('最大框架角速度 [X Y Z]：[%g %g %g] deg/s\n',r.maxRate);
fprintf('最大电机电流 [X Y Z]：[%g %g %g] A\n',r.maxCur);
fprintf('最大电机力矩 [X Y Z]：[%g %g %g] N*m\n',r.maxTor);
fprintf('限流占比 [X Y Z]：[%g %g %g] %%\n',100*r.satRatio);
fprintf('末时刻位置：纬度 %.9f deg，经度 %.9f deg，高度 %.3f m\n', ...
    rad2deg(out.lat(end)),rad2deg(out.lon(end)),out.h(end));
fprintf('末时刻ENU速度：[%g %g %g] m/s\n',out.vel(end,:));
fprintf('末时刻姿态[横滚 俯仰 航向]：[%g %g %g] deg\n', ...
    rad2deg(out.att(end,:)));
fprintf('=============================================\n\n');

%% 5. 绘图与保存
if ~exist('result','dir'), mkdir('result'); end

f1=figure('Color','w','Position',[80 70 1450 850]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
nexttile; plot(r.t,rad2deg(r.ang),'LineWidth',1); grid on;
ylabel('框架角 / deg'); legend('外环X','内环Y','台体Z');
title('三轴框架稳定回路');
nexttile; plot(r.t,rad2deg(r.rate),'LineWidth',1); grid on;
ylabel('角速度 / (deg/s)');
nexttile; plot(r.t,r.cur,'LineWidth',1); grid on;
xlabel('时间 / s'); ylabel('电流 / A');

f2=figure('Color','w','Position',[80 70 1450 800]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile; plot(r.t,r.tor,'LineWidth',1); grid on;
ylabel('电机力矩 / (N*m)'); legend('X','Y','Z'); title('执行机构输出');
nexttile; stairs(r.t,double(r.sat),'LineWidth',1); grid on;
xlabel('时间 / s'); ylabel('限流状态'); ylim([-0.1 1.1]);

f3=figure('Color','w','Position',[80 70 1450 850]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
nexttile; plot(out.t,out.vel,'LineWidth',1); grid on;
ylabel('ENU速度 / (m/s)'); legend('东','北','天'); title('导航解算输出');
nexttile; plot(out.t,[rad2deg(out.lat-out.lat(1))*3600, ...
    rad2deg(out.lon-out.lon(1))*3600,out.h-out.h(1)],'LineWidth',1); grid on;
ylabel('位置变化 / arcsec,m'); legend('纬度','经度','高度');
nexttile; plot(out.t,rad2deg(out.att),'LineWidth',1); grid on;
xlabel('时间 / s'); ylabel('姿态 / deg'); legend('横滚','俯仰','航向');

f4=figure('Color','w','Position',[100 90 1450 720]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile; plot(tn,rad2deg(cmdNav)*3600,'LineWidth',1); grid on;
ylabel('轴速率指令 / (arcsec/s)'); legend('外环X','内环Y','台体Z');
title('导航层到三轴稳定回路的反馈指令');
nexttile; plot(out.t,rad2deg(out.alpha)*3600,'LineWidth',1.2); grid on;
xlabel('时间 / s'); ylabel('游移角 / arcsec');

exportgraphics(f1,'result/loop.png','Resolution',220);
exportgraphics(f2,'result/motor.png','Resolution',220);
exportgraphics(f3,'result/nav.png','Resolution',220);
exportgraphics(f4,'result/command.png','Resolution',220);
save('result/data.mat','r','-v7.3');
fprintf('结果已保存到：%s\n',fullfile(pwd,'result'));

function R=rx(a), R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)]; end
function R=ry(a), R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)]; end
function R=rz(a), R=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1]; end
function R=rv2dcm(v)
a=norm(v);
if a<1e-14, R=eye(3)+skew(v); return; end
u=v/a; K=skew(u); R=eye(3)+sin(a)*K+(1-cos(a))*(K*K);
end
function v=dcm2rv(R)
c=max(-1,min(1,(trace(R)-1)/2)); a=acos(c);
if a<1e-10
    v=0.5*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
else
    v=a/(2*sin(a))*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
end
end
function S=skew(v), S=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0]; end
function g=normalGravity(lat,h)
g=9.7803253359*(1+0.00193185265241*sin(lat)^2) ...
    /sqrt(1-0.00669437999013*sin(lat)^2)-3.086e-6*h;
end

function [out,d,ang,rate,cur,tn,cpTrue]=decodeNav(simOut,p)
% 把稳定回路状态转换为惯性器件脉冲，再执行一次导航解算。
rng(p.seed,'twister');
angTs=simOut.get('ang'); rateTs=simOut.get('rate'); curTs=simOut.get('cur');
q=max(1,round(p.tn/p.ts));
idx=(1:q:numel(angTs.Time)).';
tn=angTs.Time(idx); ang=angTs.Data(idx,:);
rate=rateTs.Data(idx,:); cur=curTs.Data(idx,:);
n=numel(tn); da=zeros(n,3); dv=zeros(n,3);
temp=20+5*sin(2*pi*tn/1800);
wie=[0;p.earth.w*cos(p.nav.lat);p.earth.w*sin(p.nav.lat)];
gLocal=normalGravity(p.nav.lat,p.nav.h);
cpTrue=zeros(3,3,n);
for k=1:n
    cpTrue(:,:,k)=rx(ang(k,1))*ry(ang(k,2))*rz(ang(k,3));
end
for k=2:n
    dt=tn(k)-tn(k-1); Cp0=cpTrue(:,:,k-1); CpN=cpTrue(:,:,k);
    dCi=Cp0.'*rv2dcm(wie*dt)*CpN;
    daTrue=dcm2rv(dCi); w=daTrue/dt;
    wg=(eye(3)+diag(p.fog.sf)+p.fog.mis)*w+p.fog.bias ...
        +p.fog.temp*(temp(k)-20)+p.fog.arw/sqrt(dt).*randn(3,1);
    wg=min(max(wg,-p.fog.range),p.fog.range); da(k,:)=(wg*dt).';
    fp=CpN.'*[0;0;gLocal];
    if k>2, wd=(rate(k,:).'-rate(k-1,:).')/dt; else, wd=zeros(3,1); end
    lever=cross(wd,p.err.lever)+cross(w,cross(w,p.err.lever));
    vib=p.err.vibRect.*(rate(k,:).').^2;
    fa=(eye(3)+diag(p.acc.sf)+p.acc.mis)*fp+p.acc.bias+lever+vib ...
        +p.acc.temp*(temp(k)-20)+p.acc.vrw/sqrt(dt).*randn(3,1);
    fa=min(max(fa,-p.acc.range),p.acc.range); dv(k,:)=(fa*dt).';
end
angMeas=ang+p.enc.bias.';
angMeas=round(angMeas/p.enc.quant)*p.enc.quant;
angMeas=angMeas+(p.geo.nonorth*angMeas.').';
d.t=tn; d.da=da; d.dv=dv; d.ang=angMeas; d.cur=cur;
out=nav(d,p);
end

function qcmd=navCommand(out,ang,p)
% 导航角速度指令经X-Y-Z框架速率分配后送入三个稳定回路。
n=numel(out.t); qcmd=zeros(n,3);
for k=1:n
    [~,rn]=earthRadii(out.lat(k),p);
    v=out.vel(k,:).';
    wie=[0;p.earth.w*cos(out.lat(k));p.earth.w*sin(out.lat(k))];
    rm=earthRadii(out.lat(k),p);
    wen=[-v(2)/(rm+out.h(k));v(1)/(rn+out.h(k)); ...
         v(1)*tan(out.lat(k))/(rn+out.h(k))];
    adot=v(1)*tan(out.lat(k))/(rn+out.h(k));
    wrelN=[0;0;adot];
    Cnp=out.cpn(:,:,k).';
    wiCmd=Cnp*(wie+wen+wrelN);
    wiBase=Cnp*(wie+wen);
    wrelP=wiCmd-wiBase;
    A=xyzRateMap(ang(k,2),ang(k,3));
    qcmd(k,:)=(A\wrelP).';
end
lim=p.motor.rateMax.';
qcmd=min(max(qcmd,-lim),lim);
end

function A=xyzRateMap(y,z)
% qdot=[xdot ydot zdot]' 到平台相对载体角速度(平台系)的映射。
A=[cos(z)*cos(y), sin(z), 0; ...
  -sin(z)*cos(y), cos(z), 0; ...
   sin(y),        0,      1];
end

function [rm,rn]=earthRadii(lat,p)
q=sqrt(1-p.earth.e2*sin(lat)^2);
rn=p.earth.a/q; rm=p.earth.a*(1-p.earth.e2)/q^3;
end
