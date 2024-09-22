# coding:utf-8 
import pandas as pd
import glob
import os

# 讀取所有的 xlsx 檔案
file_paths = glob.glob('output_*.xlsx')  # 替換為你的檔案路徑

# 初始化一个空的列表来存储第二列数据
data = []
file_names = []

# 读取每个xlsx文件
for file in file_paths:
    df = pd.read_excel(file, header=None)  # 不设置表头
    second_column = df.iloc[:, 1].tolist()  # 提取第二列数据
    data.append(second_column)
    
    # 获取文件名（不包含路径和扩展名）
    file_name = os.path.splitext(os.path.basename(file))[0]
    file_names.append(file_name)

# 创建一个新的DataFrame，按列组合数据
result_df = pd.DataFrame(data).T

# 将文件名作为第一行（列头）
result_df.columns = file_names

# 将结果写入新的xlsx文件
result_df.to_excel('output.xlsx', index=False)