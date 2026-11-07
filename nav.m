function out = nav(d,p)
% 平台式惯导离线机械编排
% 输入字段：
%   d.t   Nx1 时间，s
%   d.da  Nx3 陀螺角增量，rad，平台系
%   d.dv  Nx3 加速度计速度增量，m/s，平台系
%   d.ang Nx3 框架角[X Y Z]，rad
%   d.cur Nx3 电机电流，A（可选，仅透传）
% 输出：经纬高、ENU速度、横滚俯仰航向、游移角及平台姿态。

n = numel(d.t);
lat = zeros(n,1); lon = lat; h = lat; alpha = lat;
vel = zeros(n,3); att = zeros(n,3);
cpn = zeros(3,3,n); cbn = zeros(3,3,n);

lat(1)=p.nav.lat; lon(1)=p.nav.lon; h(1)=p.nav.h;
vel(1,:)=p.nav.vel.'; alpha(1)=p.nav.alpha;
cpn(:,:,1)=rz(alpha(1));

for k=2:n
    dt=d.t(k)-d.t(k-1);
    [rm,rn]=radii(lat(k-1),p);

    % 真实平台姿态由陀螺角增量相对惯性空间传播，再扣除导航系转动。
    wie=[0;p.earth.w*cos(lat(k-1));p.earth.w*sin(lat(k-1))];
    wen=[-vel(k-1,2)/(rm+h(k-1)); ...
          vel(k-1,1)/(rn+h(k-1)); ...
          vel(k-1,1)*tan(lat(k-1))/(rn+h(k-1))];
    cpn(:,:,k)=rv2dcm(-(wie+wen)*dt)*cpn(:,:,k-1)*rv2dcm(d.da(k,:).');
    cpn(:,:,k)=orth(cpn(:,:,k));

    % 游移角方程，并构造理想游移平台到东北天的变换。
    alphaDot=vel(k-1,1)*tan(lat(k-1))/(rn+h(k-1));
    alpha(k)=alpha(k-1)+alphaDot*dt;
    cwn=rz(alpha(k));

    % 平台比力转换至ENU；采用中点近似更新速度。
    fn=cpn(:,:,k)*(d.dv(k,:).'/dt);
    gn=[0;0;-gravity(lat(k-1),h(k-1))];
    coriolis=cross(2*wie+wen,vel(k-1,:).');
    vel(k,:)=vel(k-1,:)+(fn+gn-coriolis).'*dt;

    lat(k)=lat(k-1)+vel(k,2)/(rm+h(k-1))*dt;
    lon(k)=lon(k-1)+vel(k,1)/((rn+h(k-1))*cos(lat(k-1)))*dt;
    h(k)=h(k-1)+vel(k,3)*dt;

    % 框架转序：外环X、内环Y、台体Z。
    cpb=rx(d.ang(k,1))*ry(d.ang(k,2))*rz(d.ang(k,3));
    % C_p^n为实际平台相对东北天姿态，C_p^b为平台到载体变换，
    % 因而载体到东北天：C_b^n=C_p^n*(C_p^b)^T。
    cbn(:,:,k)=cpn(:,:,k)*cpb.';
    att(k,:)=dcm2rpy(cbn(:,:,k)).';
end

out.t=d.t;
out.lat=lat; out.lon=lon; out.h=h;
out.vel=vel; out.att=att; out.alpha=alpha;
out.cpn=cpn; out.cbn=cbn;
if isfield(d,'cur'), out.cur=d.cur; end
end

function [rm,rn]=radii(lat,p)
q=sqrt(1-p.earth.e2*sin(lat)^2);
rn=p.earth.a/q;
rm=p.earth.a*(1-p.earth.e2)/q^3;
end

function g=gravity(lat,h)
g=9.7803253359*(1+0.00193185265241*sin(lat)^2) ...
    /sqrt(1-0.00669437999013*sin(lat)^2)-3.086e-6*h;
end

function C=rv2dcm(v)
a=norm(v); S=skew(v);
if a<1e-12, C=eye(3)+S; return; end
C=eye(3)+sin(a)/a*S+(1-cos(a))/a^2*(S*S);
end

function C=orth(C)
[U,~,V]=svd(C); C=U*diag([1 1 det(U*V.')])*V.';
end

function e=dcm2rpy(C)
pitch=asin(max(-1,min(1,-C(3,1))));
roll=atan2(C(3,2),C(3,3));
yaw=atan2(C(2,1),C(1,1));
e=[roll;pitch;yaw];
end

function R=rx(a), R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)]; end
function R=ry(a), R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)]; end
function R=rz(a), R=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1]; end
function S=skew(v), S=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0]; end
