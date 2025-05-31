function [] = create_cpp_LUT(file_path, x_data, y_data, v_min, pwm_min)
%Generate a C++ array file for arduino to use as a LUT

fid = fopen(file_path, 'w');
fprintf(fid, '#pragma once\n\n');

fprintf(fid, 'const int LUT_SIZE = %d;\n\n', length(x_data));

fprintf(fid, 'const int STICTION_VOLTAGE = %d;\n\n', v_min);

fprintf(fid, 'const int STICTION_PWM = %d;\n\n', pwm_min);

fprintf(fid, 'const float voltage_LUT[LUT_SIZE] = {');
fprintf(fid, '%.3f, ', x_data(1:end-1));
fprintf(fid, '%.3f};\n\n', x_data(end));

fprintf(fid, 'const float pwm_LUT[LUT_SIZE] = {');
fprintf(fid, '%.3f, ', y_data(1:end-1));
fprintf(fid, '%.3f};\n', y_data(end));

fclose(fid);

end