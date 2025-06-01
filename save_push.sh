#!/bin/bash

# 현재 경로로 이동
cd "$(dirname "$0")"

# 변경사항 추가 → 커밋 → 푸시
git add .
git commit -m "자동 커밋: $(date '+%Y-%m-%d %H:%M:%S')"
git push origin main
