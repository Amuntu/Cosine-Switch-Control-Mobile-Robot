function plot_robot(x, y, theta, robot_length, robot_width, wheel_radius, wheel_width)
   
    % Define the robot corners relative to the center
    corners = [-robot_length/2, -robot_width/2;
               robot_length/2, -robot_width/2;
               robot_length/2, robot_width/2;
               -robot_length/2, robot_width/2;
               -robot_length/2, -robot_width/2];
    
    % Define the wheel positions relative to the center
    wheel_positions = [0, -robot_width/2 - wheel_width/2;
                       0, robot_width/2 + wheel_width/2;];
    
    % Define the wheel shapes
    wheel_shapes = [-wheel_radius, -wheel_width/2;
                    wheel_radius, -wheel_width/2;
                    wheel_radius, wheel_width/2;
                    -wheel_radius, wheel_width/2;
                    -wheel_radius, -wheel_width/2];

    % Rotation matrix
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    
    % Rotate and translate the corners
    rotated_corners = (R * corners')';
    translated_corners = rotated_corners + [x, y];
    
    % Rotate and translate the wheels
    rotated_wheels = (R * wheel_positions')';
    translated_wheels = rotated_wheels + [x, y];
    
    % Rotate the wheel shapes
    rotated_wheel_shapes = (R * wheel_shapes')';
    
    % Plot the robot
    hold on;
    fill(translated_corners(:,1), translated_corners(:,2), 'b-');
    % Plot the wheels
    pos = translated_wheels(1,:);
    wheel = rotated_wheel_shapes + pos;
    fill(wheel(:,1), wheel(:,2), 'k');
    pos = translated_wheels(2,:);
    wheel = rotated_wheel_shapes + pos;
    fill(wheel(:,1), wheel(:,2), 'k');
    % Plot the orientation
    quiver(x, y, cos(theta), sin(theta), robot_width/2, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    quiver(x, y, cos(theta + pi/2), sin(theta + pi/2), robot_width/2, 'g', 'LineWidth', 2, 'MaxHeadSize', 2);    
    hold off;
end


