function AllanDemo()

% 仿真白噪声,RMS = 2.0,带宽BW=200Hz,那么对应的sqrt(PSD)=RMS/sqrt(BW)
bw = 200;              % 采样率，带宽
time = 1:1/bw:10000;   % 总时长 10000 s
rms = 2.0;             % 均方根
white = rms * randn(length(time),1);    % 白噪声序列


% 仿真随机游走，游走系数与前面白噪声q（PSD）一致
rw = cumsum(white);

% 仿真正弦波
omega = 0.01;
s = sin(time * 2*pi*omega);

close all; 
[av, tau] = Allan(white, 1/bw, 1.2);
AllanPlot(tau, av);

[av1, tau1] = Allan(rw, 1/bw, 1.2);
AllanPlot(tau1, av1);

[av2, tau2] = Allan(s, 1/bw, 1.02);
AllanPlot(tau2, av2);

function AllanPlot(tau, data)
figure,
loglog(tau,data,'-r','LineWidth',2,...
    'MarkerFaceColor','k','MarkerSize',8);  %设置线型，颜色，宽度
xlabel('\tau (s)','fontsize',15);
ylabel('\sigma(\tau)','fontsize',15,'Rotation',90);
% title('Allan deviation','fontsize',15);
grid on
axis equal 
