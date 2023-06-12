classdef BoomController
    
    properties
        Node
        GaitPublisher
        LegPointPublisher
        MotorCommandPublisher
    end
    
    methods
        function obj = BoomController()
            obj.Node = ros2node('boom_controller', 2);
            obj.GaitPublisher = ros2publisher(obj.Node,...
                '/gait', 'std_msgs/String');
            obj.LegPointPublisher = ros2publisher(obj.Node,...
                '/legs/cmd', 'sensor_msgs/PointCloud');
            obj.MotorCommandPublisher = ros2publisher(obj.Node,...
                '/motors/cmd', 'trajectory_msgs/JointTrajectoryPoint');
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
