#!/bin/bash

# 获取 ../ 的绝对路径
BASE_PATH=$(realpath ../)

# 绝对路径需要替换的路径
OLD_ABS_PATH="/home/lty/ustc/catkin_ws"

# 打印即将替换的信息
echo "This script will replace all occurrences of '$OLD_ABS_PATH' with '$BASE_PATH' in files under '../src'."

# 询问是否继续
read -p "Do you want to continue? (y/n): " CONFIRM

if [[ $CONFIRM != [Yy] ]]; then
  echo "Operation cancelled."
  exit 1
fi

# 遍历 ../src 目录下的所有文件
find ../src -type f | while read -r FILE; do
  # 检查文件中是否包含绝对路径
  if grep -q "$OLD_ABS_PATH" "$FILE"; then
    echo "Processing $FILE"

    # 打印文件中要替换的内容
    echo "Content containing '$OLD_ABS_PATH' in $FILE:"
    grep "$OLD_ABS_PATH" "$FILE"
    
    # 提示用户确认替换
    # read -p "Do you want to replace '$OLD_ABS_PATH' with '$BASE_PATH' in $FILE? (y/n): " CONFIRM
    
    # if [[ $CONFIRM == [Yy] ]]; then
      # 使用 sed 命令替换绝对路径为新的路径
    sed -i "s|$OLD_ABS_PATH|$BASE_PATH|g" "$FILE"
    echo "Replaced paths in $FILE"
    # else
    #   echo "Skipped $FILE"
    # fi
  fi
done

echo "All files processed."
