#!/usr/bin/env bash
# 本脚本只负责编译 ESP-IDF 工程，不执行烧录或串口监视。
set -euo pipefail

# 通过脚本自身位置解析项目根目录，允许从任意工作目录调用。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
IDF_ROOT="${IDF_PATH:-/opt/esp/idf}"

# 加载 ESP-IDF 工具链，并把生成物隔离到 build/output。
source "${IDF_ROOT}/export.sh"
cd "${PROJECT_ROOT}"
idf.py -B build/output build
