classdef BoomController
    
    properties
        Node
        GaitPublisher
        LegPointPublisher
        MotorCommandPublisher
        MotorEncoderData
        MotorEncoderErrorData
        MotorVoltageData
        MotorCurrentData
        MotorErrorData
        EncoderDataSubscriber
        EncoderErrorDataSubscriber
        VoltageDataSubscriber
        CurrentDataSubscriber
        ErrorDataSubscriber
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

            obj.MotorEncoderData = [];
            obj.MotorEncoderErrorData = [];
            obj.MotorVoltageData = [];
            obj.MotorCurrentData = [];
            obj.MotorErrorData = 0;

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
        end

        function obj = encoder_callback(obj, msg)
            obj.MotorEncoderData = [obj.MotorEncoderData, msg.data.'];
        end

        function obj = voltage_callback(obj, msg)
            obj.MotorVoltageData = [obj.MotorVoltageData, msg.data.'];
        end

        function obj = current_callback(obj, msg)
            obj.MotorCurrentData = [obj.MotorCurrentData, msg.data.'];
        end

        function obj = encoder_error_callback(obj, msg)
            obj.MotorEncoderErrorData = [obj.MotorEncoderErrorData, msg.data.'];
        end

        function obj = error_callback(obj, msg)
            obj.MotorErrorData = msg.data.';
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
    end
end

