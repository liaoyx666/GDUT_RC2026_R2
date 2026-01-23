folderPath = 'D:/code/my_RC/my_data3.csv';%路径
fileList = dir(fullfile(folderPath,'*.csv'));
% 初始化存储结果的变量
varResults = [];  % 存储每个文件的方差
fileNameList = {};  % 存储对应的文件名

% 循环处理每个文件
for i = 1:length(fileList)
    % 获取当前文件名
    fileName = fullfile(folderPath, fileList(i).name);
    fileNameList{i} = fileList(i).name;  % 记录文件名
    
    % 读取Excel文件的A列数据（假设数据从A1开始，无标题行）
    % 若有标题行，range改为'A2:A'（读取A2及以下数据）
    data = readmatrix(fileName);  % 读取A列所有数据
    data = data(:,1);  % 提取A列（第一列）数据

    % 过滤空值（若数据中有空单元格）
    data = data(~isnan(data));
    
    % 计算方差（var函数默认计算样本方差，加参数1计算总体方差）
    dataVar = var(data);  % 样本方差（除以n-1）
    % dataVar = var(data, 1);  % 总体方差（除以n）
    
    % 保存当前文件的方差
    varResults = [varResults; dataVar];
end
% 组合文件名和方差为表格
resultTable = table(fileNameList', varResults, 'VariableNames', {'FileName', 'Variance'});
% 保存到新Excel文件
%writetable(fullfile(folderPath, '方差结果汇总.xlsx'), resultTable);