function [temp,ax,ay,az,mx,my,mz,gx,gy,gz] = tti(filename, startRow, endRow)
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
%   importfile('Temperature_bi_1431887714.3.txt',1, 73883);
%
%    See also TEXTSCAN.

% Auto-generated by MATLAB on 2015/05/17 21:00:51

%% Initialize variables.
delimiter = ',';
if nargin<=2
    startRow = 1;
    endRow = inf;
end

%% Read columns of data as strings:
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%s%s%s%s%s%s%s%s%s%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
textscan(fileID, '%[^\n\r]', startRow(1)-1, 'ReturnOnError', false);
dataArray = textscan(fileID, formatSpec, endRow(1)-startRow(1)+1, 'Delimiter', delimiter, 'ReturnOnError', false);
for block=2:length(startRow)
    frewind(fileID);
    textscan(fileID, '%[^\n\r]', startRow(block)-1, 'ReturnOnError', false);
    dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, 'Delimiter', delimiter, 'ReturnOnError', false);
    for col=1:length(dataArray)
        dataArray{col} = [dataArray{col};dataArrayBlock{col}];
    end
end

%% Close the text file.
fclose(fileID);

%% Convert the contents of columns containing numeric strings to numbers.
% Replace non-numeric strings with NaN.
raw = repmat({''},length(dataArray{1}),length(dataArray)-1);
for col=1:length(dataArray)-1
    raw(1:length(dataArray{col}),col) = dataArray{col};
end
numericData = NaN(size(dataArray{1},1),size(dataArray,2));

for col=[1,2,3,4,5,6,7,8,9,10]
    % Converts strings in the input cell array to numbers. Replaced non-numeric
    % strings with NaN.
    rawData = dataArray{col};
    for row=1:size(rawData, 1);
        % Create a regular expression to detect and remove non-numeric prefixes and
        % suffixes.
        regexstr = '(?<prefix>.*?)(?<numbers>([-]*(\d+[\,]*)+[\.]{0,1}\d*[eEdD]{0,1}[-+]*\d*[i]{0,1})|([-]*(\d+[\,]*)*[\.]{1,1}\d+[eEdD]{0,1}[-+]*\d*[i]{0,1}))(?<suffix>.*)';
        try
            result = regexp(rawData{row}, regexstr, 'names');
            numbers = result.numbers;
            
            % Detected commas in non-thousand locations.
            invalidThousandsSeparator = false;
            if any(numbers==',');
                thousandsRegExp = '^\d+?(\,\d{3})*\.{0,1}\d*$';
                if isempty(regexp(thousandsRegExp, ',', 'once'));
                    numbers = NaN;
                    invalidThousandsSeparator = true;
                end
            end
            % Convert numeric strings to numbers.
            if ~invalidThousandsSeparator;
                numbers = textscan(strrep(numbers, ',', ''), '%f');
                numericData(row, col) = numbers{1};
                raw{row, col} = numbers{1};
            end
        catch me
        end
    end
end


%% Exclude rows with non-numeric cells
J = ~all(cellfun(@(x) isnumeric(x) || islogical(x),raw),2); % Find rows with non-numeric cells
raw(J,:) = [];

%% Exclude rows with blank cells
J = any(cellfun(@(x) isempty(x) || (ischar(x) && all(x==' ')),raw),2); % Find row with blank cells
raw(J,:) = [];

%% Exclude columns with non-numeric cells
I = ~all(cellfun(@(x) isnumeric(x) || islogical(x),raw),1); % Find columns with non-numeric cells
raw(:,I) = [];

%% Exclude columns with blank cells
I = any(cellfun(@(x) isempty(x) || (ischar(x) && all(x==' ')),raw),1); % Find columns with blank cells
raw(:,I) = [];

temp = [];
ax = [];
ay = [];
az = [];
mx = [];
my = [];
mz = [];
gx = [];
gy = [];
gz = [];
%% Initialize column outputs.
columnIndices = cumsum(~I);

%% Allocate imported array to column variable names
if ~I(1)
    temp = cell2mat(raw(:, columnIndices(1)));
end
if ~I(2)
    ax = cell2mat(raw(:, columnIndices(2)));
end
if ~I(3)
    ay = cell2mat(raw(:, columnIndices(3)));
end
if ~I(4)
    az = cell2mat(raw(:, columnIndices(4)));
end
if ~I(5)
    mx = cell2mat(raw(:, columnIndices(5)));
end
if ~I(6)
    my = cell2mat(raw(:, columnIndices(6)));
end
if ~I(7)
    mz = cell2mat(raw(:, columnIndices(7)));
end
if ~I(8)
    gx = cell2mat(raw(:, columnIndices(8)));
end
if ~I(9)
    gy = cell2mat(raw(:, columnIndices(9)));
end
if ~I(10)
    gz = cell2mat(raw(:, columnIndices(10)));
end

