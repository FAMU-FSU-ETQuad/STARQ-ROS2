classdef BoomController < handle
    
    properties
        Node

        GaitPublisher
        LegPointPublisher
        MotorCommandPublisher
        MotorPGainPublisher
        MotorVGainPublisher
        MotorIGainPublisher

        MotorEncoderData
        MotorEncoderErrorData
        MotorVoltageData
        MotorCurrentData
        MotorErrorData
        MotorTemperatureData

        MotorEncoderDataEvent
        MotorEncoderErrorDataEvent
        MotorVoltageDataEvent
        MotorCurrentDataEvent
        MotorErrorDataEvent
        MotorTemperatureDataEvent

        EncoderDataSubscriber
        EncoderErrorDataSubscriber
        VoltageDataSubscriber
        CurrentDataSubscriber
        ErrorDataSubscriber
        TemperatureDataSubscriber
    end

    events
        MotorEncoderDataRecieved
        MotorEncoderErrorDataRecieved
        MotorVoltageDataRecieved
        MotorCurrentDataRecieved
        MotorErrorDataRecieved
        MotorTemperatureDataRecieved
    end
    
    methods
        function obj = BoomController()

            obj.Node = ros2node('boom_controller');

            obj.GaitPublisher = ros2publisher(obj.Node,...
                '/gait', 'std_msgs/String');
            obj.LegPointPublisher = ros2publisher(obj.Node,...
                '/legs/cmd', 'sensor_msgs/PointCloud');
            obj.MotorCommandPublisher = ros2publisher(obj.Node,...
                '/motors/cmd', 'trajectory_msgs/JointTrajectoryPoint');
            obj.MotorPGainPublisher = ros2publisher(obj.Node,...
                '/motors/gains/pos', 'std_msgs/Float32MultiArray');
            obj.MotorVGainPublisher = ros2publisher(obj.Node,...
                '/motors/gains/vel', 'std_msgs/Float32MultiArray');
            obj.MotorIGainPublisher = ros2publisher(obj.Node,...
                '/motors/gains/vel_int', 'std_msgs/Float32MultiArray');

            obj.MotorEncoderData = [];
            obj.MotorEncoderErrorData = [];
            obj.MotorVoltageData = [];
            obj.MotorCurrentData = [];
            obj.MotorErrorData = 0;

            obj.MotorEncoderDataEvent = event.EventData();
            obj.MotorEncoderErrorDataEvent = event.EventData();
            obj.MotorVoltageDataEvent = event.EventData();
            obj.MotorCurrentDataEvent = event.EventData();
            obj.MotorErrorDataEvent = event.EventData();
            obj.MotorTemperatureDataEvent = event.EventData();

            obj.EncoderDataSubscriber = ros2subscriber(obj.Node,...
                '/motors/info/encoders', 'std_msgs/Float32MultiArray',...
               @(msg) obj.encoder_callback(msg));
            obj.VoltageDataSubscriber = ros2subscriber(obj.Node,...
                '/motors/info/bus_voltage', 'std_msgs/Float32MultiArray',...
               @(msg) obj.voltage_callback(msg));
            obj.CurrentDataSubscriber = ros2subscriber(obj.Node,...
                '/motors/info/bus_current', 'std_msgs/Float32MultiArray',...
               @(msg) obj.current_callback(msg));
            obj.EncoderErrorDataSubscriber = ros2subscriber(obj.Node,...
                '/motors/info/encoders_error', 'std_msgs/Float32MultiArray',...
               @(msg) obj.encoder_error_callback(msg));
            obj.ErrorDataSubscriber = ros2subscriber(obj.Node,...
                '/motors/info/errors', 'std_msgs/Float32MultiArray',...
               @(msg) obj.error_callback(msg));
            obj.TemperatureDataSubscriber = ros2subscriber(obj.Node,...
                '/motors/info/temperature', 'std_msgs/Float32MultiArray',...
               @(msg) obj.temperature_callback(msg));

            addlistener(obj, 'MotorEncoderDataRecieved', @obj.handle_encoder_data);
            addlistener(obj, 'MotorVoltageDataRecieved', @obj.handle_voltage_data);
            addlistener(obj, 'MotorCurrentDataRecieved', @obj.handle_current_data);
            addlistener(obj, 'MotorEncoderErrorDataRecieved', @obj.handle_encoder_error_data);
            addlistener(obj, 'MotorErrorDataRecieved', @obj.handle_error_data);
            addlistener(obj, 'MotorTemperatureDataRecieved', @obj.handle_temperature_data);
        end
        
        function send_gait(obj,gait_name)
            gait_msg = ros2message(obj.GaitPublisher);
            gait_msg.data = char(gait_name);
            send(obj.GaitPublisher, gait_msg)
        end

        function send_leg_point(obj,leg_point)
            point3d = zeros(1,3);
            point3d(1:numel(leg_point)) = leg_point;
            point_msg = ros2message(obj.LegPointPublisher);
            point_msg.points(1).x = single(point3d(1));
            point_msg.points(1).y = single(point3d(2));
            point_msg.points(1).z = single(point3d(3));
            send(obj.LegPointPublisher, point_msg)
        end

        function send_motor_command(obj,motor_positions)
            motor_cmd_msg = ros2message(obj.MotorCommandPublisher);
            motor_cmd_msg.positions = motor_positions;
            send(obj.MotorCommandPublisher, motor_cmd_msg);
        end

        function send_leg_trajectory(obj, point_trajectory, publish_rate)
            period = 1 / publish_rate;
            tic
            for p = 1:size(point_trajectory,2)
                obj.send_leg_point(point_trajectory(p,:));
                pause(p*period - toc)
            end
        end

        function send_motor_trajectory(obj, motor_trajectory, publish_rate)
            period = 1 / publish_rate;
            tic
            for m = 1:size(motor_trajectory,2)
                obj.send_motor_command(motor_trajectory(m,:));
                pause(m*period - toc)
            end
        end

        function encoder_callback(obj, msg)
            obj.MotorEncoderDataEvent.Data = msg.data;
            notify(obj, 'MotorEncoderDataRecieved', obj.MotorEncoderDataEvent)
        end

        function handle_encoder_data(obj, ~, data)
            obj.MotorEncoderData = [obj.MotorEncoderData, data];
        end

        function voltage_callback(obj, msg)
            obj.MotorVoltageDataEvent.Data = msg.data;
            notify(obj, 'MotorVoltageDataRecieved', obj.MotorVoltageDataEvent)
        end

        function handle_voltage_data(obj, ~, data)
            obj.MotorVoltageData = [obj.MotorVoltageData, data];
        end

        function current_callback(obj, msg)
            obj.MotorCurrentDataEvent.Data = msg.data;
            notify(obj, 'MotorCurrentDataRecieved', obj.MotorCurrentDataEvent)
        end

        function handle_current_data(obj, ~, data)
            obj.MotorCurrentData = [obj.MotorCurrentData, data];
        end

        function encoder_error_callback(obj, msg)
            obj.MotorEncoderErrorDataEvent.Data = msg.data;
            notify(obj, 'MotorEncoderErrorDataRecieved', obj.MotorEncoderErrorDataEvent)
        end

        function handle_encoder_error_data(obj, ~, data)
            obj.MotorEncoderErrorData = [obj.MotorEncoderErrorData, data];
        end

        function error_callback(obj, msg)
            obj.MotorErrorDataEvent.Data = msg.data;
            notify(obj, 'MotorErrorDataRecieved', obj.MotorErrorDataEvent)
        end

        function handle_error_data(obj, ~, data)
            obj.MotorErrorData = [obj.MotorErrorData, data];
        end

        function temperature_callback(obj, msg)
            obj.MotorTemperatureDataEvent.Data = msg.data;
            notify(obj, 'MotorTemperatureDataRecieved', obj.MotorTemperatureDataEvent)
        end

        function handle_temperature_data(obj, ~, data)
            obj.MotorTemperatureData = [obj.MotorTemperatureData, data];
        end
    end
end

