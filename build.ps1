[CmdletBinding()]
param(
    [switch]$Bootstrap,
    [switch]$Validate,
    [switch]$CheckShaders,
    [switch]$Deliver,
    [switch]$UiDevServer
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$repository = $PSScriptRoot
Set-Location -LiteralPath $repository
$utf8 = [Text.UTF8Encoding]::new($false)
if ($Deliver -and $UiDevServer) { throw 'A delivery embeds the built interface; -UiDevServer is for development only.' }
$dependencies = Get-Content -Raw -LiteralPath 'dependencies.json' | ConvertFrom-Json
$dependency = $dependencies.'llvm-mingw'
$deps = Join-Path $repository 'build/deps'
$toolchain = Join-Path $deps $dependency.directory
if ($env:AZP_CAMERA_TOOLCHAIN) { $toolchain = $env:AZP_CAMERA_TOOLCHAIN }
$webview2 = $dependencies.'webview2-sdk'
$webview2Include = Join-Path $deps ($webview2.directory + '/build/native/include')
$json = $dependencies.'nlohmann-json'
$jsonInclude = Join-Path $deps $json.directory
$ui = Join-Path $repository 'ui'
$uiStamp = Join-Path $ui 'node_modules/.azp-package-lock.sha256'
$uiLock = (Get-FileHash -LiteralPath (Join-Path $ui 'package-lock.json')).Hash

# Downloads $Url to $Path through a .partial file. The pinned digest is mandatory.
function Save-Pinned([string]$Url, [string]$Sha256, [string]$Path) {
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $Path) | Out-Null
    if (-not (Test-Path -LiteralPath $Path)) {
        Invoke-WebRequest -Uri $Url -OutFile ($Path + '.partial')
        if ((Get-FileHash -LiteralPath ($Path + '.partial')).Hash -ine $Sha256) { throw "Hash mismatch: $Url" }
        Move-Item -LiteralPath ($Path + '.partial') -Destination $Path
    }
    if ((Get-FileHash -LiteralPath $Path).Hash -ine $Sha256) { throw "Cached file hash mismatch: $Path" }
}

# Download only on explicit request: the compiler, the WebView2 SDK headers, nlohmann/json and the locked npm
# packages of ui/.
if ($Bootstrap) {
    if (-not (Test-Path -LiteralPath (Join-Path $toolchain 'bin/clang++.exe'))) {
        if ($env:AZP_CAMERA_TOOLCHAIN) { throw 'AZP_CAMERA_TOOLCHAIN does not contain the compiler.' }
        $archive = Join-Path $deps 'llvm-mingw.zip'
        Save-Pinned $dependency.url $dependency.sha256 $archive
        Expand-Archive -LiteralPath $archive -DestinationPath (Split-Path -Parent $toolchain) -Force
    }
    if (-not (Test-Path -LiteralPath (Join-Path $webview2Include 'WebView2.h'))) {
        $archive = Join-Path $deps 'webview2-sdk.zip'
        Save-Pinned $webview2.url $webview2.sha256 $archive
        Expand-Archive -LiteralPath $archive -DestinationPath (Join-Path $deps $webview2.directory) -Force
    }
    Save-Pinned $json.url $json.sha256 (Join-Path $jsonInclude 'nlohmann/json.hpp')
    if (-not (Test-Path -LiteralPath $uiStamp) -or ([IO.File]::ReadAllText($uiStamp)).Trim() -ne $uiLock) {
        & npm.cmd --prefix $ui ci --no-audit --no-fund
        if ($LASTEXITCODE -ne 0) { throw 'npm ci failed in ui/.' }
        [IO.File]::WriteAllText($uiStamp, $uiLock, $utf8)
    }
}
$compiler = Join-Path $toolchain 'bin/clang++.exe'
$windres = Join-Path $toolchain 'bin/x86_64-w64-mingw32-windres.exe'
if (-not (Test-Path -LiteralPath $compiler)) { throw 'Set AZP_CAMERA_TOOLCHAIN or run build.ps1 -Bootstrap.' }
if (-not (Test-Path -LiteralPath (Join-Path $webview2Include 'WebView2.h')) -or -not (Test-Path -LiteralPath (Join-Path $jsonInclude 'nlohmann/json.hpp'))) {
    throw 'The WebView2 SDK or nlohmann/json headers are missing: run build.ps1 -Bootstrap.'
}
if (-not (Test-Path -LiteralPath $uiStamp) -or ([IO.File]::ReadAllText($uiStamp)).Trim() -ne $uiLock) {
    throw 'ui/node_modules does not match ui/package-lock.json: run build.ps1 -Bootstrap.'
}
$compilerLines = @(& $compiler --version)
$compilerVersion = $compilerLines[0]
if ($LASTEXITCODE -ne 0 -or $compilerVersion -notmatch 'clang version 23\.1\.1') { throw 'This repository requires pinned LLVM 23.1.1.' }
$version = (Get-Content -Raw -LiteralPath 'version.json' | ConvertFrom-Json).version
$protocolHeader = Get-Content -Raw -LiteralPath 'src/shared/protocol.hpp'
if ($protocolHeader -notmatch 'ProtocolVersion\s*=\s*(\d+)') { throw 'Missing EXE/DLL protocol version.' }
$protocol = [uint32]$Matches[1]
if ($version -notmatch '^(\d+)\.(\d+)\.(\d+)$') { throw 'version.json must contain major.minor.patch.' }
$major=$Matches[1]; $minor=$Matches[2]; $patch=$Matches[3]
$out = Join-Path $repository 'build/native'
$generated = Join-Path $out 'generated'
New-Item -ItemType Directory -Force -Path $generated | Out-Null
$versionHeader = "#pragma once`n#define AZP_CAMERA_VERSION `"$version`"`n#define AZP_CAMERA_VERSION_MAJOR $major`n#define AZP_CAMERA_VERSION_MINOR $minor`n#define AZP_CAMERA_VERSION_PATCH $patch`n#define AZP_CAMERA_BUILD_NUMBER 0`n"
[IO.File]::WriteAllText((Join-Path $generated 'azp-camera-version.hpp'), $versionHeader, $utf8)
# Every local variable starts at zero, so no code path depends on leftover stack contents.
$common = @('-std=c++20','-O2','-ftrivial-auto-var-init=zero','-Wall','-Wextra','-Werror','-fms-extensions','-static',
    '-DNOMINMAX','-DWIN32_LEAN_AND_MEAN','-D_WIN32_WINNT=0x0A00','-mno-avx','-mno-avx2','-mno-avx512f','-I',$generated)
# The EXE window also uses the WebView2 SDK and nlohmann/json headers, as system headers.
$exeFlags = @('-isystem',$webview2Include,'-isystem',$jsonInclude)
if ($UiDevServer) { $exeFlags += '-DAZP_CAMERA_UI_DEV_SERVER' }
$libs = @('-ld3d12','-ldxgi','-ldxguid','-ld3dcompiler','-lversion')
# The EXE is src/exe/ plus src/shared/ (a Windows application without console); the DLL is everything else.
# Each has one entry file.
$app = Get-Item -LiteralPath 'src/exe/app.cpp'
$bridge = Get-Item -LiteralPath 'src/dll/dll_main.cpp'
# Source discovery is limited to src/. Tests under tests/ are never linked into the EXE or DLL.
$sources = @(Get-ChildItem -LiteralPath 'src' -File -Recurse | Where-Object { $_.Extension -in @('.cpp','.S') -and $_.FullName -notin @($app.FullName,$bridge.FullName) } | Sort-Object FullName)
$headers = @(Get-ChildItem -LiteralPath 'src' -File -Recurse -Filter '*.hpp' | Sort-Object FullName)

$headerKey = $compilerVersion + $versionHeader + ($common -join ' ') + (($headers | ForEach-Object { (Get-FileHash -LiteralPath $_.FullName).Hash }) -join '')
$exeKey = ($exeFlags -join ' ') + $webview2.version + $json.version
$objects = @()
$appObjects = @()
Write-Output "Building AZP Camera $version with LLVM 23.1.1."
foreach ($source in $sources) {
    $relative = $source.FullName.Substring($repository.Length + 1)
    $object = Join-Path $out (($relative -replace '[/\\]','_') + '.o')
    $cache = $object + '.inputs'
    $flags = if ($relative -like 'src\exe\*') { $common + $exeFlags } else { $common }
    $key = $headerKey + (Get-FileHash -LiteralPath $source.FullName).Hash + $(if ($relative -like 'src\exe\*') { $exeKey } else { '' })
    if (-not (Test-Path -LiteralPath $object) -or -not (Test-Path -LiteralPath $cache) -or [IO.File]::ReadAllText($cache) -cne $key) {
        & $compiler @flags '-c' $source.FullName '-o' $object
        if ($LASTEXITCODE -ne 0) { throw "Native compilation failed: $relative" }
        [IO.File]::WriteAllText($cache, $key, $utf8)
    }
    if ($relative -notlike 'src\exe\*') { $objects += $object }
    if ($relative -like 'src\exe\*' -or $relative -like 'src\shared\*') { $appObjects += $object }
}
& $compiler @common '-shared' $bridge.FullName @objects @libs '-Wl,--no-insert-timestamp' '-o' (Join-Path $out 'azp-camera.dll')
if ($LASTEXITCODE -ne 0) { throw 'Native DLL link failed.' }

# The window's interface: ui/ built by Vite into one HTML file, embedded in the EXE with the icon. The corner
# picture is embedded at 1.5 times its displayed height (224 px, ui/src/styles/_app.scss), not at the size of
# assets/background.png.
$headerSource = Join-Path $repository 'assets/background.png'
$headerScript = Join-Path $repository 'tools/resize-image.ps1'
$headerImage = Join-Path $ui 'src/generated/background.png'
$headerHeight = 336
$headerKey = (Get-FileHash -LiteralPath $headerSource).Hash + (Get-FileHash -LiteralPath $headerScript).Hash + $headerHeight
if (-not (Test-Path -LiteralPath $headerImage) -or -not (Test-Path -LiteralPath ($headerImage + '.inputs')) -or [IO.File]::ReadAllText($headerImage + '.inputs') -cne $headerKey) {
    & $headerScript -Source $headerSource -Destination $headerImage -Height $headerHeight
    [IO.File]::WriteAllText($headerImage + '.inputs', $headerKey, $utf8)
}
& npm.cmd --prefix $ui run build
if ($LASTEXITCODE -ne 0) { throw 'Interface build failed (ui/).' }
Copy-Item -LiteralPath (Join-Path $ui 'dist/index.html') -Destination (Join-Path $generated 'azp-camera-ui.html') -Force
$iconSource = Join-Path $repository 'assets/branding/azp-camera-icon.png'
$iconScript = Join-Path $repository 'tools/generate-windows-icon.ps1'
$icon = Join-Path $generated 'azp-camera.ico'
$iconKey = (Get-FileHash -LiteralPath $iconSource).Hash + (Get-FileHash -LiteralPath $iconScript).Hash
if (-not (Test-Path -LiteralPath $icon) -or -not (Test-Path -LiteralPath ($icon + '.inputs')) -or [IO.File]::ReadAllText($icon + '.inputs') -cne $iconKey) {
    & $iconScript -Source $iconSource -Destination $icon
    [IO.File]::WriteAllText($icon + '.inputs', $iconKey, $utf8)
}
$resources = Join-Path $out 'src_exe_resources.rc.o'
& $windres '-I' $generated '-I' (Join-Path $repository 'src/exe') '-i' (Join-Path $repository 'src/exe/resources.rc') '-O' 'coff' '-o' $resources
if ($LASTEXITCODE -ne 0) { throw 'EXE resource compilation failed.' }
& $compiler @common @exeFlags '-municode' '-mwindows' $app.FullName @appObjects $resources '-ladvapi32' '-lshell32' '-luser32' '-lgdi32' '-lole32' '-ldwmapi' '-Wl,--no-insert-timestamp' '-o' (Join-Path $out 'AZP-Camera.exe')
if ($LASTEXITCODE -ne 0) { throw 'EXE build failed.' }
Copy-Item -LiteralPath (Join-Path $repository 'THIRD_PARTY_NOTICES.txt') -Destination (Join-Path $out 'THIRD_PARTY_NOTICES.txt') -Force
Copy-Item -LiteralPath (Join-Path $repository 'LICENCE.md') -Destination (Join-Path $out 'LICENCE.md') -Force
if ($CheckShaders) { & (Join-Path $repository 'tests/compile-shaders.ps1') -Repository $repository }
# Tests are opt-in. No test executable runs during a normal build/delivery.
if ($Validate) {
    & (Join-Path $repository 'tests/run.ps1') -Compiler $compiler -Generated $generated
}
if ($Deliver) {
    $tag = 'AZP-Camera-' + (Get-Date -Format 'yyyyMMdd-HHmmss')
    $destination = Join-Path $repository ('build/deliveries/' + $tag)
    New-Item -ItemType Directory -Path $destination | Out-Null
    foreach ($name in @('AZP-Camera.exe','azp-camera.dll','LICENCE.md','THIRD_PARTY_NOTICES.txt')) { Copy-Item -LiteralPath (Join-Path $out $name) -Destination (Join-Path $destination $name) }
    $receipt = [ordered]@{version=$version;directory=$destination;native_build_passed=$true;shader_compile_passed=[bool]$CheckShaders;protocol=$protocol;tests_executed=[bool]$Validate;simulator_validation='pending user';files=@(Get-ChildItem -LiteralPath $destination -File | ForEach-Object { [pscustomobject]@{name=$_.Name;bytes=$_.Length;sha256=(Get-FileHash -LiteralPath $_.FullName).Hash} });source_files=@(Get-ChildItem -LiteralPath 'src' -File -Recurse | Sort-Object FullName | ForEach-Object { [pscustomobject]@{path=$_.FullName.Substring($repository.Length+1);sha256=(Get-FileHash -LiteralPath $_.FullName).Hash} })}
    $receiptDirectory = Join-Path $repository 'build/validation/deliveries'

    New-Item -ItemType Directory -Force -Path $receiptDirectory | Out-Null
    $receipt | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $receiptDirectory ($tag+'.json')) -Encoding utf8
    Write-Output "Delivery: $destination"
}
Write-Output "Native build complete: $out"
