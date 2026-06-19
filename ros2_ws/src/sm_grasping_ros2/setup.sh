#!/bin/bash
# setup.sh - 선택적 venv 구성, 의존성 설치, GPD 소스 준비
set -e

echo "의존성 설치를 시작합니다."
USE_SYSTEM_PYTHON=1

if [ ! -d "venv" ] || [ ! -f "venv/bin/activate" ]; then
  if [ -d "venv" ] && [ ! -f "venv/bin/activate" ]; then
    echo "손상된 venv 폴더를 감지했습니다. 기존 venv를 무시하고 다시 시도합니다."
  fi
  read -p "가상환경(venv)을 생성하시겠습니까? (y/n): " create_venv
  if [ "$create_venv" = "y" ]; then
    if python3 -m venv venv; then
      source venv/bin/activate
      USE_SYSTEM_PYTHON=0
    else
      echo "가상환경 생성에 실패했습니다. 시스템 Python으로 계속 진행합니다."
      echo "Ubuntu/Debian에서는 다음 패키지가 필요할 수 있습니다: python3-venv"
    fi
  fi
else
  source venv/bin/activate
  USE_SYSTEM_PYTHON=0
fi

PIP_FLAGS=""
if [ "$USE_SYSTEM_PYTHON" -eq 1 ]; then
  PIP_FLAGS="--break-system-packages"
fi

python3 -m pip install --upgrade pip $PIP_FLAGS
python3 -m pip install -r requirements.txt $PIP_FLAGS

read -p "GPD 소스를 다운로드하시겠습니까? (y/n): " install_gpd
if [ "$install_gpd" = "y" ]; then
  GPD_DIR_DEFAULT="$(pwd)/third_party/gpd"
  read -p "GPD 경로를 입력하세요 (기본값: ${GPD_DIR_DEFAULT}): " gpd_dir_input
  GPD_DIR="${gpd_dir_input:-$GPD_DIR_DEFAULT}"

  if [ ! -d "$GPD_DIR" ]; then
    mkdir -p "$(dirname "$GPD_DIR")"
    git clone https://github.com/atenpas/gpd.git "$GPD_DIR"
  fi

  echo "GPD 소스 준비 완료: $GPD_DIR"
  echo "GPD는 C++ 빌드가 필요합니다. 아래를 참고해 별도로 빌드하세요."
  echo "  cd $GPD_DIR && mkdir -p build && cd build && cmake .. && make -j\$(nproc)"
fi

if [ ! -f .env ] && [ -f .env.example ]; then
  cp .env.example .env
  echo ".env 파일을 생성했습니다. 필요한 값을 채워주세요."
fi

echo "환경 설치가 완료되었습니다."
