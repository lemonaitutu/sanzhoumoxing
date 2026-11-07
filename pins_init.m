function p = pins_init
% Simulink模型初始化：自动加载参数，并为缺失输入建立零信号。
p=par;
assignin('base','p',p);

t=[0;p.stop];
z=timeseries([0;0],t);
names={'mx','my','mz','cx','cy','cz'};
for k=1:numel(names)
    name=names{k};
    if evalin('base',sprintf("exist('%s','var')",name))==0
        assignin('base',name,z);
    end
end
end
