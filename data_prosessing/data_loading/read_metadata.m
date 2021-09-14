function [json_data] = read_metadata(filename)
    str = fileread(filename);
    json_data = jsondecode(str);
end