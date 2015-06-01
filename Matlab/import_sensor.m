function [temp,ax,ay,az,mx,my,mz,gx,gy,gz] = import_sensor(filename, startRow, endRow)
%IMPORTFILE Import numeric data from a text file as column vectors.
%   [TEMP,AX,AY,AZ,MX,MY,MZ,GX,GY,GZ] = IMPORTFILE(FILENAME) Reads data
%   from text file FILENAME for the default selection.
%
%   [TEMP,AX,AY,AZ,MX,MY,MZ,GX,GY,GZ] = IMPORTFILE(FILENAME, STARTROW,
%   ENDROW) Reads data from rows STARTROW through ENDROW of text file
%   FILENAME.
%
% Example:
%   [temp,ax,ay,az,mx,my,mz,gx,gy,gz] =
%   importfile('Serial_1432141657.57_kj_upperarm.txt',22, 35641);
%
%    See also TEXTSCAN.

% Auto-generated by MATLAB on 2015/05/20 20:08:06

%% Initialize variables.
delimiter = ',';
if nargin<=2
    startRow = 22;
    endRow = inf;
end

%% Format string for each line of text:
%   column1: text (%s)
%	column2: text (%s)
%   column3: text (%s)
%	column4: text (%s)
%   column5: text (%s)
%	column6: text (%s)
%   column7: text (%s)
%	column8: text (%s)
%   column9: text (%s)
%	column10: text (%s)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, endRow(1)-startRow(1)+1, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines', startRow(1)-1, 'ReturnOnError', false);
for block=2:length(startRow)
    frewind(fileID);
    dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines', startRow(block)-1, 'ReturnOnError', false);
    for col=1:length(dataArray)
        dataArray{col} = [dataArray{col};dataArrayBlock{col}];
    end
end

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
temp = dataArray{:, 1};
ax = dataArray{:, 2};
ay = dataArray{:, 3};
az = dataArray{:, 4};
mx = dataArray{:, 5};
my = dataArray{:, 6};
mz = dataArray{:, 7};
gx = dataArray{:, 8};
gy = dataArray{:, 9};
gz = dataArray{:, 10};

