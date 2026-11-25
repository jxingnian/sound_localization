# 本脚本只负责可重复编译，不执行烧录，也不会探测或占用任何串口。
$ErrorActionPreference = "Stop"

# 脚本固定放在 build/sound_localization 下，因此向上两级就是项目根目录。
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$idfRoot = "D:\esp\v6.1-beta1\esp-idf"

# 使用项目当前 sdkconfig 对应的 ESP-IDF 6.1 环境，保证编译器和头文件版本一致。
. (Join-Path $idfRoot "export.ps1")
Push-Location $projectRoot
try {
    # 输出写入 build/output，避免清理时删除本构建脚本。
    idf.py -B build/output build
    if ($LASTEXITCODE -ne 0) {
        throw "ESP-IDF 编译失败，退出码：$LASTEXITCODE"
    }
}
finally {
    Pop-Location
}
