function [structs_scans] = collectScans_modified(positions, headings, reps)
try
    rosinit();
catch
    disp("Ros already initialized")
end

% Create empty structure array for all scans
struct_scan = struct();
struct_scan.r_all = [];
struct_scan.theta_all = [];
struct_scan.posn = [];
struct_scan.heading = [];
structs_scans = repelem(struct_scan, length(headings), 1);

for i = 1 : length(headings)
    posn = positions(i, :);
    head_x = cos(headings(i));
    head_y = sin(headings(i));
    f_placeNeato = @() placeNeato(posn(1), posn(2), head_x, head_y, 0.25);

    sub = rossubscriber('/scan');

    % place Neato at the origin pointing in the ihat_G direction
    f_placeNeato();

    % wait a while for the Neato to fall into place
    pause(0.5);

    % Collect data at the room origin
    r_all = [];
    theta_all = [];
    for j = 1 : reps
        scan_message = receive(sub);
        r_j = scan_message.Ranges(1:end-1);
        theta_j = deg2rad([0:359]');
        
        r_all = [r_all r_j];
        theta_all = [theta_all theta_j];
    end
    
    % Save this scan session into a struct
    struct_scan = struct();
    struct_scan.r_all = r_all;
    struct_scan.theta_all = theta_all;
    struct_scan.posn = posn;
    struct_scan.heading = headings(i);
    
    structs_scans(i) = struct_scan;
end

rosshutdown();
end
