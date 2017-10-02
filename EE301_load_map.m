%EE301_load_map
% Copyright: Dr. Travis Dierks

figure_mapmaker=figure;
hold on;

if get_file_name

[filename pathname filter]= uigetfile('*.txt','Import Map File');
end
if filename                         % Make sure cancel was not pressed
    fid= fopen([filename]);% Get file handle for parsing

    % Clear old map
    children= get(gca,'Children');  % All ploted items on axes
    if ~isempty(children)
        delete(children)	% Delete all but robot from axes
    end
    mapMat= {[]};

   % Parse the file and extract relevant information
    walls= [];
    lines= [];
    beacs= {};
    vwalls= [];
    while ~feof(fid)
        line= fgetl(fid);
        line= lower(line);  % Convert to lowercase if necessary
        line= strtrim(line);    % Delete leading and trailing whitespace
        mapMat= [mapMat ; line];% Save to map cell array
        lineWords= {};          % To keep track of line entries
        while ~isempty(line) && ~strcmp(line(1),'%')% End of line or comment
            [word line]= strtok(line);  % Get next entry
            line= strtrim(line);        % To be able to detect comments
            lineWords= [lineWords word];
        end

        %%% Regular expressions would probably be more efficient %%%
        if length(lineWords) == 5 && strcmp(lineWords{1},'wall') && ...
                ~isnan(str2double(lineWords{2})) && ...
                ~isnan(str2double(lineWords{3})) && ...
                ~isnan(str2double(lineWords{4})) && ...
                ~isnan(str2double(lineWords{5}))
            walls= [walls ; str2double(lineWords{2}) ...
                str2double(lineWords{3}) str2double(lineWords{4}) ...
                str2double(lineWords{5})];
        elseif length(lineWords) == 5 && strcmp(lineWords{1},'line') && ...
                ~isnan(str2double(lineWords{2})) && ...
                ~isnan(str2double(lineWords{3})) && ...
                ~isnan(str2double(lineWords{4})) && ...
                ~isnan(str2double(lineWords{5}))
            lines= [lines ; str2double(lineWords{2}) ...
                str2double(lineWords{3}) str2double(lineWords{4}) ...
                str2double(lineWords{5})];
        elseif length(lineWords) == 7 && strcmp(lineWords{1},'beacon') ...
                && ~isnan(str2double(lineWords{2})) && ...
                ~isnan(str2double(lineWords{3})) && ...
                length(lineWords{4}) >= 2 && ...
                strcmp(lineWords{4}(1),'[') && ...
                ~isnan(str2double(lineWords{4}(2:end))) && ...
                ~isnan(str2double(lineWords{5})) && ...
                length(lineWords{6}) >= 2 && ...
                strcmp(lineWords{6}(end),']') && ...
                ~isnan(str2double(lineWords{6}(1:end-1)))
            beacs= [beacs ; str2double(lineWords{2}) ...
                str2double(lineWords{3}) ...
                str2double(lineWords{4}(2:end)) ...
                str2double(lineWords{5}) ...
                str2double(lineWords{6}(1:end-1)) lineWords(7)];
        elseif length(lineWords) == 5 && ...
                strcmp(lineWords{1},'virtwall') && ...
                ~isnan(str2double(lineWords{2})) && ...
                ~isnan(str2double(lineWords{3})) && ...
                ~isnan(str2double(lineWords{4})) && ...
                ~isnan(str2double(lineWords{5})) && ...
                str2double(lineWords{5}) >= 1 && ...
                str2double(lineWords{5}) <= 3
            vwalls= [vwalls ; str2double(lineWords{2}) ...
                str2double(lineWords{3}) str2double(lineWords{4}) ...
                str2double(lineWords{5})];
        elseif ~isempty(lineWords)
            warning('MATLAB:invalidInput',...
                'This line in map file %s is unrecognized:\n\t%s',...
                filename,line)
        end
    end
    fclose(fid);

    % Store map data, replacing old data
     set(figure_mapmaker,'UserData',mapMat)

    % Plot walls
    for i= 1:size(walls,1)
        plot(walls(i,[1 3]),walls(i,[2 4]),'k-','LineWidth',1)
    end

    % Plot lines
    for i= 1:size(lines,1)
        plot(lines(i,[1 3]),lines(i,[2 4]),'k--','LineWidth',1)
    end

    % Plot beacons
    for i= 1:size(beacs,1)
        plot(beacs{i,1},beacs{i,2},...
            'Color',cell2mat(beacs(i,3:5)),'Marker','o')
        text(beacs{i,1},beacs{i,2},['  ' beacs{i,6}])
    end

    % Define virtual wall emitter constants
    halo_rad= 0.45;     % Radius of the halo around the emitter
    range_short= 2.13;  % Range of the wall on the 0-3' setting
    ang_short= 0.33;    % Angular range on the 0-3' setting
    range_med= 5.56;    % Range of the wall on the 4'-7' setting
    ang_med= 0.49;      % Angular range on the 4'-7' setting
    range_long= 8.08;   % Range of the wall on the 8'+ setting
    ang_long= 0.61;     % Angular range on the 8'+ setting

    % Get points to map virtual walls
    for i= 1:size(vwalls,1)
        x_vw= vwalls(i,1);
        y_vw= vwalls(i,2);
        th_vw= vwalls(i,3);
        if vwalls(i,4) == 1
            range_vw= range_short;
            ang_vw= ang_short;
        elseif vwalls(i,4) == 2
            range_vw= range_med;
            ang_vw= ang_med;
        else
            range_vw= range_long;
            ang_vw= ang_long;
        end
        x_1= x_vw+range_vw*cos(th_vw+ang_vw/2);
        y_1= y_vw+range_vw*sin(th_vw+ang_vw/2);
        x_2= x_vw+range_vw*cos(th_vw-ang_vw/2);
        y_2= y_vw+range_vw*sin(th_vw-ang_vw/2);

        % Plot halo around emitter and range triangle
        circ_numPts= 21;    % Estimate circle as circ_numPts-1 lines
        circ_ang=linspace(0,2*pi,circ_numPts);
        circ_rad=ones(1,circ_numPts)*halo_rad;
        [circ_x circ_y]= pol2cart(circ_ang,circ_rad);
        circ_x= circ_x+x_vw;
        circ_y= circ_y+y_vw;
        plot(x_vw,y_vw,'g*')
        plot(circ_x,circ_y,'g:','LineWidth',1);
        plot([x_vw x_1 x_2 x_vw],[y_vw y_1 y_2 y_vw],'g:','LineWidth',1)
    end
end
