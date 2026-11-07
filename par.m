function p = par
% 三轴平台式惯导数字样机参数
% 轴顺序固定为：[外环X，内环Y，台体Z]。

%% 仿真
p.ts = 1/2000;                    % 稳定回路周期，s（根据笔记）
p.tn = 0.01;                      % 导航解算周期，s（暂定100 Hz）
p.stop = 60;                      % 默认仿真时长，s
p.seed = 21;
p.ideal = strcmp(getenv('PINS_IDEAL'),'1'); % 1：关闭传感器误差，检查机械编排闭合
p.loop.iter = 2;                  % 导航-稳定回路协同迭代次数
p.loop.relax = 0.7;               % 指令松弛系数，避免迭代振荡

%% 三轴框架及力矩电机
p.ax(1) = axisPar('X',0.047268,0.60,7.6e-4,100e-3,6e-3);
p.ax(2) = axisPar('Y',0.020106,0.48,5.1e-4, 60e-3,4e-3);
p.ax(3) = axisPar('Z',0.014540,0.48,5.1e-4, 60e-3,2e-3);

% 电机力矩系数相对误差（笔记给出±7%）。
p.motor.ktErr = [0.03;-0.02;0.04];

% 经验暂定值，后续用实测数据替换。
p.motor.iMax = [1.5;1.5;1.5];     % 最大电流，A
p.motor.uMax = [28;28;28];        % 母线电压，V
p.motor.rateMax = deg2rad([120;140;180]); % 最大角速度，rad/s
p.motor.accMax = deg2rad([450;600;800]);  % 最大角加速度，rad/s^2

%% 三轴共同校正网络
p.ctrl.kc = 191;                  % 笔记参数，待最终确认
p.ctrl.gcNum = [0.0028 1];
p.ctrl.gcDen = [0.0040 1];
p.ctrl.kp = 0.4;                  % 功率级增益，待最终确认

% PID作为独立可调环节；第一版以比例为主，积分、微分采用保守暂定值。
p.ctrl.pid = [1.0,8.0,2e-4];      % [P I D]
p.ctrl.n = 500;                   % 微分滤波系数

%% 光纤陀螺
p.fog.bw = 100;                   % 一阶带宽，Hz（暂定）
p.fog.tau = 1/(2*pi*p.fog.bw);
p.fog.fs = 1/p.ts;
p.fog.delay = 1*p.ts;             % 1个采样周期延迟（暂定）
p.fog.bias = deg2rad([0.01;-0.008;0.006])/3600; % rad/s
p.fog.biasStd = deg2rad([0.003;0.003;0.003])/3600;
p.fog.arw = deg2rad([0.002;0.002;0.002])/sqrt(3600);
p.fog.sf = [80;-60;70]*1e-6;
p.fog.mis = deg2rad([0 8 -6;-7 0 9;5 -8 0]/3600);
p.fog.range = deg2rad(400);        % rad/s
p.fog.temp = deg2rad([0.002;-0.0015;0.001])/3600; % (rad/s)/degC，暂定

%% 加速度计
p.acc.fs = 1/p.tn;
p.acc.bw = 50;                    % Hz，暂定
p.acc.delay = p.tn;
p.acc.bias = [40;-35;50]*1e-6*9.80665;
p.acc.biasStd = [8;8;10]*1e-6*9.80665;
p.acc.vrw = [30;30;40]*1e-6*9.80665;
p.acc.sf = [150;-120;180]*1e-6;
p.acc.mis = deg2rad([0 12 -9;-10 0 11;8 -13 0]/3600);
p.acc.range = 20*9.80665;
p.acc.temp = [8;-6;10]*1e-6*9.80665; % (m/s^2)/degC，暂定

%% 框架几何、编码器及附加误差
p.geo.nonorth = deg2rad([0 12 -8;-10 0 15;7 -11 0]/3600);
p.enc.bias = deg2rad([5;-7;4]/3600);
p.enc.quant = deg2rad(1/3600);
p.enc.fs = 1/p.ts;

% 杆臂：平台中心到加速度计测量中心，平台坐标系，m。
p.err.lever = [0.035;-0.022;0.018];
p.err.vibRect = [5;5;8]*1e-6;     % 振动整流系数，暂定
p.err.backlash = deg2rad([3;3;2]/3600); % 框架间隙，rad，暂定
p.err.viscous = [8;6;5]*1e-4;     % N*m*s/rad，暂定
p.err.coulomb = [2.0;1.5;1.2]*1e-3; % N*m，暂定

%% 旋转变压器/角度测量
p.rdc.wn = 114*2*pi;              % rad/s（根据笔记）
p.rdc.zeta = 0.707;

%% 地球、游移方位与初始导航状态
p.earth.a = 6378137;
p.earth.f = 1/298.257223563;
p.earth.e2 = p.earth.f*(2-p.earth.f);
p.earth.w = 7.292115e-5;
p.earth.g0 = 9.80665;

p.nav.lat = deg2rad(39.9);
p.nav.lon = deg2rad(116.4);
p.nav.h = 50;
p.nav.vel = [0;0;0];              % ENU，m/s
p.nav.alpha = 0;                  % 初始游移角，rad
p.nav.cp2b = eye(3);              % 初始平台到载体姿态

% 理想闭合模式只关闭测量/几何误差，不改变三轴稳定回路本体。
if p.ideal
    p.fog.bias(:)=0; p.fog.arw(:)=0; p.fog.sf(:)=0;
    p.fog.mis(:)=0; p.fog.temp(:)=0;
    p.acc.bias(:)=0; p.acc.vrw(:)=0; p.acc.sf(:)=0;
    p.acc.mis(:)=0; p.acc.temp(:)=0;
    p.geo.nonorth(:)=0; p.enc.bias(:)=0;
    p.err.lever(:)=0; p.err.vibRect(:)=0;
end

end

function a = axisPar(name,J,kt,tg,dist,imb)
a.name = name;
a.J = J;
a.kt = kt;
a.tg = tg;
a.distMax = dist;
a.imbMax = imb;
end
