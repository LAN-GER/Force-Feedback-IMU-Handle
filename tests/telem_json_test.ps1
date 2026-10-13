param([string]$VisualStudio = 'C:/Program Files/Microsoft Visual Studio/2022/Community')
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$toolset = Get-ChildItem "$VisualStudio/VC/Tools/MSVC" -Directory | Sort-Object Name -Descending |
    Where-Object { (Test-Path "$($_.FullName)/bin/HostX86/x86/cl.exe") -and (Test-Path "$($_.FullName)/lib/x86/libcmt.lib") } |
    Select-Object -First 1
if (-not $toolset) { throw 'A complete MSVC x86 toolset is required' }
Import-Module "$VisualStudio/Common7/Tools/Microsoft.VisualStudio.DevShell.dll"
Enter-VsDevShell -VsInstallPath $VisualStudio -Arch x86 -HostArch x86 -SkipAutomaticLocation -DevCmdArguments "-vcvars_ver=$($toolset.Name)"
if (-not (Get-Command cl.exe -ErrorAction SilentlyContinue)) { throw 'MSVC cl.exe is required' }
$output = Join-Path $repo 'build/host_telem'
New-Item -ItemType Directory -Force -Path $output | Out-Null
& cl.exe /nologo /utf-8 /std:c11 /O2 /W4 /WX "/I$repo/tests/host_stubs" "/I$repo/software/components/svc/include" "/I$repo/software/components/drv/include" "/Fo$output/telem_json_test.obj" "/Fe$output/telem_json_test.exe" "$repo/tests/telem_json_test.c"
if ($LASTEXITCODE -ne 0) { throw 'Native serializer compilation failed' }
& "$output/telem_json_test.exe"
if ($LASTEXITCODE -ne 0) { throw 'Native serializer tests failed' }
