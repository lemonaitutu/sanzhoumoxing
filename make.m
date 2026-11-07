function make
% 生成三轴平台稳定回路Simulink模型。

mdl='model';
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end
new_system(mdl);
set_param(mdl,'Solver','ode4','SolverType','Fixed-step', ...
    'FixedStep','p.ts','StopTime','p.stop', ...
    'SignalLogging','on','SaveOutput','on', ...
    'InitFcn','pins_init;');

% 外部干扰力矩与导航轴速率指令输入。
src={'mx','my','mz'};
cmd={'cx','cy','cz'};
for i=1:3
    add_block('simulink/Sources/From Workspace',[mdl '/' upper(src{i})], ...
        'VariableName',src{i},'Interpolate','on', ...
        'Position',[25 80+210*(i-1) 130 110+210*(i-1)]);
    add_block('simulink/Sources/From Workspace',[mdl '/' upper(cmd{i})], ...
        'VariableName',cmd{i},'Interpolate','on', ...
        'Position',[25 130+210*(i-1) 130 160+210*(i-1)]);
    addAxis([mdl '/' upper(char('W'+i))],i,[190 35+210*(i-1) 900 185+210*(i-1)]);
    add_line(mdl,[upper(src{i}) '/1'],[upper(char('W'+i)) '/1'],'autorouting','on');
    add_line(mdl,[upper(cmd{i}) '/1'],[upper(char('W'+i)) '/2'],'autorouting','on');
end

% 各轴输出组合成[X Y Z]向量。
names={'ang','rate','cur','tor','sat','raw'};
for j=1:numel(names)
    y=70+95*(j-1);
    add_block('simulink/Signal Routing/Mux',[mdl '/Mux_' names{j}], ...
        'Inputs','3','Position',[975 y 980 y+45]);
    add_block('simulink/Sinks/To Workspace',[mdl '/Log_' names{j}], ...
        'VariableName',names{j},'SaveFormat','Timeseries', ...
        'Position',[1040 y+5 1150 y+35]);
    for i=1:3
        add_line(mdl,[upper(char('W'+i)) '/' num2str(j)], ...
            ['Mux_' names{j} '/' num2str(i)],'autorouting','on');
    end
    add_line(mdl,['Mux_' names{j} '/1'],['Log_' names{j} '/1']);
end

set_param(mdl,'Location',[100 80 1320 820]);
save_system(mdl,[mdl '.slx']);
close_system(mdl);
fprintf('已生成：%s\n',fullfile(pwd,[mdl '.slx']));
end

function addAxis(path,index,pos)
add_block('simulink/Ports & Subsystems/Subsystem',path,'Position',pos);

% 清理Subsystem默认端口。
delete_line(path,'In1/1','Out1/1');
delete_block([path '/In1']); delete_block([path '/Out1']);

add_block('simulink/Ports & Subsystems/In1',[path '/Md'], ...
    'Position',[20 78 50 92]);
add_block('simulink/Ports & Subsystems/In1',[path '/Cmd'], ...
    'Port','2','Position',[20 143 50 157]);
add_block('simulink/Math Operations/Sum',[path '/Sum'], ...
    'Inputs','+-','Position',[80 70 105 100]);
add_block('simulink/Math Operations/Gain',[path '/InvJ'], ...
    'Gain',sprintf('1/p.ax(%d).J',index),'Position',[135 70 195 100]);
add_block('simulink/Continuous/Integrator',[path '/Rate'], ...
    'Position',[225 70 255 100]);
add_block('simulink/Continuous/Integrator',[path '/Angle'], ...
    'Position',[300 70 330 100]);

add_block('simulink/Continuous/Transfer Fcn',[path '/FOG'], ...
    'Numerator','1','Denominator','[p.fog.tau 1]', ...
    'Position',[300 145 380 175]);
add_block('simulink/Math Operations/Sum',[path '/RateErr'], ...
    'Inputs','+-','Position',[385 140 410 180]);
add_block('simulink/Continuous/Transfer Fcn',[path '/Gc'], ...
    'Numerator','p.ctrl.gcNum','Denominator','p.ctrl.gcDen', ...
    'Position',[430 145 500 175]);
add_block('simulink/Math Operations/Gain',[path '/Kc'], ...
    'Gain','p.ctrl.kc','Position',[520 145 570 175]);
add_block('simulink/Continuous/PID Controller',[path '/PID'], ...
    'P','p.ctrl.pid(1)','I','p.ctrl.pid(2)','D','p.ctrl.pid(3)', ...
    'N','p.ctrl.n','Position',[600 135 680 180]);
add_block('simulink/Math Operations/Gain',[path '/Kp'], ...
    'Gain','p.ctrl.kp','Position',[710 145 760 175]);
add_block('simulink/Discontinuities/Saturation',[path '/Ilim'], ...
    'UpperLimit',sprintf('p.motor.iMax(%d)',index), ...
    'LowerLimit',sprintf('-p.motor.iMax(%d)',index), ...
    'Position',[790 140 835 180]);
add_block('simulink/Continuous/Transfer Fcn',[path '/Motor'], ...
    'Numerator',sprintf('p.ax(%d).kt*(1+p.motor.ktErr(%d))',index,index), ...
    'Denominator',sprintf('[p.ax(%d).tg 1]',index), ...
    'Position',[865 140 945 180]);

% 旋转变压器只作用于框架角输出，不改变陀螺稳定回路。
add_block('simulink/Continuous/Transfer Fcn',[path '/RDC'], ...
    'Numerator','p.rdc.wn^2', ...
    'Denominator','[1 2*p.rdc.zeta*p.rdc.wn p.rdc.wn^2]', ...
    'Position',[385 40 485 70]);

% 限流状态：abs(Iraw-I)>1e-9。
add_block('simulink/Math Operations/Sum',[path '/Idiff'], ...
    'Inputs','+-','Position',[850 220 875 250]);
add_block('simulink/Math Operations/Abs',[path '/Abs'], ...
    'Position',[895 220 925 250]);
add_block('simulink/Sources/Constant',[path '/Tol'], ...
    'Value','1e-9','Position',[900 275 930 295]);
add_block('simulink/Logic and Bit Operations/Relational Operator',[path '/Sat'], ...
    'Operator','>','Position',[955 225 990 255]);

outName={'ang','rate','cur','tor','sat','raw'};
for j=1:6
    add_block('simulink/Ports & Subsystems/Out1',[path '/' outName{j}], ...
        'Port',num2str(j),'Position',[1035 25+48*(j-1) 1065 39+48*(j-1)]);
end

add_line(path,'Md/1','Sum/1');
add_line(path,'Sum/1','InvJ/1');
add_line(path,'InvJ/1','Rate/1');
add_line(path,'Rate/1','Angle/1');
add_line(path,'Rate/1','FOG/1','autorouting','on');
add_line(path,'FOG/1','RateErr/1');
add_line(path,'Cmd/1','RateErr/2','autorouting','on');
add_line(path,'RateErr/1','Gc/1');
add_line(path,'Gc/1','Kc/1');
add_line(path,'Kc/1','PID/1');
add_line(path,'PID/1','Kp/1');
add_line(path,'Kp/1','Ilim/1');
add_line(path,'Ilim/1','Motor/1');
add_line(path,'Motor/1','Sum/2','autorouting','on');
add_line(path,'Angle/1','RDC/1');

add_line(path,'Kp/1','Idiff/1','autorouting','on');
add_line(path,'Ilim/1','Idiff/2','autorouting','on');
add_line(path,'Idiff/1','Abs/1');
add_line(path,'Abs/1','Sat/1');
add_line(path,'Tol/1','Sat/2');

add_line(path,'RDC/1','ang/1');
add_line(path,'Rate/1','rate/1','autorouting','on');
add_line(path,'Ilim/1','cur/1','autorouting','on');
add_line(path,'Motor/1','tor/1','autorouting','on');
add_line(path,'Sat/1','sat/1');
add_line(path,'Kp/1','raw/1','autorouting','on');
end
