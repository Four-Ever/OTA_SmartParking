#!/bin/bash

# 현재 사용자의 모든 프로세스 종료
killall -u $USER

# 또는 더 안전한 방법:
for pid in $(ps -u $USER -o pid=); do
    if [ $pid != $$ ]; then  # 현재 실행 중인 스크립트는 제외
        kill -15 $pid 2>/dev/null || true
    fi
done

echo "모든 프로세스가 종료되었습니다."