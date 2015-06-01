function [temp,ax,ay,az,mx,my,mz,gx,gy,gz] = temperature_text_import(filename, startRow, endRow)
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
%   importfile('Temperature_bi_1431778286.93.txt',1, 1526);
%
%    See also TEXTSCAN.

% Auto-generated by MATLAB on 2015/05/16 17:35:11

%% Initialize variables.
delimiter = ',';
if nargin<2
    startRow = 1;
    endRow = inf;
end
if nargin<3
    endRow = inf;
end

%% Format string for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
%	column8: double (%f)
%   column9: double (%f)
%	column10: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, endRow(1)-startRow(1)+1, 'Delimiter', delimiter, 'HeaderLines', startRow(1)-1, 'ReturnOnError', false);
for block=2:length(startRow)
    frewind(fileID);
    dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, 'Delimiter', delimiter, 'HeaderLines', startRow(block)-1, 'ReturnOnError', false);
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

