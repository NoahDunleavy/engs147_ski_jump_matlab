function results = parse_arduino_log(file_path)
%From an arduino log, following our specified format, parse out collected
%info, and macro headers

fid = fopen(file_path, 'r'); %read in file

%Entry format:
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %Header: %d
    %Header: %d
    %Data format: col_name,col_name,col_name
    %-------------------------------------------------------
    %[Date follows here, as presented in format]
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++

lines = {}; %declare cell array for lines
current_line = fgetl(fid); %read one line at a time

while ischar(current_line) %while the line is good
    lines{end+1} = strtrim(current_line);  %add to array
    current_line = fgetl(fid); %advance line
end
fclose(fid); %close file

%Initialize structure
results = struct('data', {}, 'headers', {}, 'test_info', {});


inside_data_block = false; %transition between tests
inside_header_block = false;

data_lines = {};
test_info = struct('desc', {}, 'value', {});
format_headers = {};

for ndx = 1:length(lines) %for every line
    line = lines{ndx}; %grab the line

    if contains(line, '+++++') %if we hit a boundary line
        if inside_data_block %if we have already been bringing in data, means we are at end
            numeric_rows = cellfun(@(row) str2double(row), data_lines, 'UniformOutput', false); %cell function, for each row, comvert string to double
            numeric_matrix = cell2mat(numeric_rows); %cells to matrix

            result.data = numeric_matrix;
            result.headers = format_headers;
            result.test_info = test_info;
            
            results(end+1) = result; %expand results list as needed

            data_lines = {}; %reset data line
            test_info = struct('desc', {}, 'value', {});
            format_headers = {};
            inside_data_block = false;  %mark we have exitted
            
        else    %if not in a data block, means we have just entered one
            inside_header_block = true;

        end
        continue
    end

    if inside_header_block
        if contains(line, 'Data format:')   %specific date format line
            line_split = split(line, ':'); %split at :

            format_headers = strip(split(line_split(2), ','));
        elseif contains(line, ':') %all other header lines
            line_split = split(line, ':');

            desc = strip(line_split{1});
            val_str = strip(line_split{2});
            val_num = str2double(val_str);
            if isnan(val_num)
                val = val_str;
            else
                val = val_num;
            end
            test_info(end+1).desc = desc;
            test_info(end).value = val;

            % header_value = str2double(line_split(2));
            % header_label = line_split(1);
        elseif contains(line, '---') %end of header block
            inside_header_block = false;
            inside_data_block = true;
            continue;
        else
            fprintf('%s does not match any patterns in the header block\n', line)
        end
    end

    if inside_data_block
        data_lines{end+1, 1} = split(line, ',')'; %add in the csv line
    end
end

end