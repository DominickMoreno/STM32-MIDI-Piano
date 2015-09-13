clc
clear all
close all

note = @(x, f) 2047 * sin(2 * pi * f .* x) + 2048;

freq = 523.25/4;
%freq = 659.1;
%freq = 783.99;
%freq = 932;
t = linspace(0, 8*pi, 64);
start = 2; 
fin = start + 31;
t_ = t(start:fin);

waveform = note(t_, freq);
hexForm = dec2hex(round(waveform));
size(hexForm)
str = '';
for ind = 1:32
    hexVal = '0x';
    hexVal = [hexVal, num2str(hexForm(ind, 1)), num2str(hexForm(ind, 2)), num2str(hexForm(ind, 3)), ','];
    str = strcat(str, hexVal);
end

plot(t_, waveform);

