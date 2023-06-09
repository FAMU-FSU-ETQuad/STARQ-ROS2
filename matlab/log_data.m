clc
clear
close all
%%

boom = BoomController();

boom.send_gait('jump')
pause(1)
encoder_data = boom.MotorEncoderData;
encoder_error_data = boom.MotorEncoderErrorData;
voltage_data = boom.MotorVoltageData;
current_data = boom.MotorCurrentData;

% Close node
clear boom