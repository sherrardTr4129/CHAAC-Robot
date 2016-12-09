% Axis [LX, LY, Trigger, RX, RY]
joy = vrjoystick(1);
previous = instrfind('Type','serial');
if(~isempty(previous))
    fclose(previous);
end
port = serial('COM39', 'BaudRate', 9600, 'Databits', 8, 'Parity', 'none', ...
    'Stopbits', 1, 'Timeout', 5000);
pause(0.5);
fopen(port)
pause(0.5);

while(1)
    var = read(joy);
    b1 = button(joy, 1)
    b2 = button(joy, 4)
    LeftY = 200*(1 + var(2))
    RightY = 200*(1+ var(5))
    if b1 == 1
        fprintf(port, 1)
    end
    if b2 == 1
        fprintf(port, 2)
    end
    fprintf(port,  strcat('l', num2str((LeftY))))
    fprintf(port,  strcat('r', num2str((RightY))))
    pause(.1)
end
fclose(port)
