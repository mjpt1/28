# ESP-IDF 6.0.1 environment without export.ps1.
# export.ps1 fails because gdb / riscv / openocd were not installed (not needed for ESP32-S3 builds).
# Usage:  . D:\job\28\idf6_env.ps1   then   idf build / idf flash ...
$T = 'C:\Espressif\tools'
$env:IDF_TOOLS_PATH = 'C:\Espressif'
$env:IDF_PATH = 'C:\Espressif\frameworks\esp-idf-v6.0.1'
$env:ESP_IDF_VERSION = '6.0'
$env:IDF_PYTHON_ENV_PATH = 'C:\Espressif\python_env\idf6.0_py3.11_env'
$env:ESP_ROM_ELF_DIR = (Get-ChildItem "$T\esp-rom-elfs" -Directory | Select-Object -First 1).FullName + '\'
$bins = @(
    "$env:IDF_PYTHON_ENV_PATH\Scripts",
    "$T\xtensa-esp-elf\esp-15.2.0_20251204\xtensa-esp-elf\bin",
    "$T\cmake\4.0.3\bin"
)
foreach ($n in 'ninja', 'idf-exe', 'ccache') {
    $bins += (Get-ChildItem "$T\$n" -Directory | Select-Object -First 1).FullName
}
$bins += (Get-ChildItem "$T\esp32ulp-elf" -Recurse -Directory -Filter bin | Select-Object -First 1).FullName
$env:PATH = ($bins -join ';') + ';' + $env:PATH
function idf { python "$env:IDF_PATH\tools\idf.py" @args }
