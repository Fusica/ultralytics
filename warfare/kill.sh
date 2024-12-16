#!/bin/bash

# 列出当前所有会话
echo "当前运行的会话："
tmux ls

# 关闭所有会话
echo -e "\n正在关闭所有会话..."
tmux kill-session -t siyi 2>/dev/null
tmux kill-session -t cam 2>/dev/null
tmux kill-session -t mavros 2>/dev/null
tmux kill-session -t lidar 2>/dev/null
tmux kill-session -t detect 2>/dev/null

# 验证是否所有会话都已关闭
sleep 1
remaining_sessions=$(tmux ls 2>/dev/null)
if [ -z "$remaining_sessions" ]; then
    echo "所有会话已成功关闭"
else
    echo "警告：以下会话仍在运行："
    echo "$remaining_sessions"
fi