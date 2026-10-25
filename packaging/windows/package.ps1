[CmdletBinding()]
param(
    [string]$Repository = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path,
    [string]$BuildDirectory,
    [string]$OutputDirectory,
    [string]$CacheDirectory
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if (-not $IsWindows) { throw 'Windows packaging must run on Windows with PowerShell 7.' }
$Repository = (Resolve-Path $Repository).Path
$work = $null
if (-not $BuildDirectory) { $BuildDirectory = Join-Path $Repository 'target/release' }
if (-not $OutputDirectory) { $OutputDirectory = Join-Path $Repository 'dist' }
if (-not $CacheDirectory) {
    $temporary = if ($env:RUNNER_TEMP) { $env:RUNNER_TEMP } else { [IO.Path]::GetTempPath() }
    $CacheDirectory = Join-Path $temporary 'flursys-gmsh-cache'
}
$BuildDirectory = [IO.Path]::GetFullPath($BuildDirectory, $Repository)
$OutputDirectory = [IO.Path]::GetFullPath($OutputDirectory, $Repository)
$CacheDirectory = [IO.Path]::GetFullPath($CacheDirectory, $Repository)
function Receive-VerifiedFile([string]$Url, [string]$Hash, [string]$Destination) {
    if (Test-Path $Destination) {
        if ((Get-FileHash $Destination -Algorithm SHA256).Hash -ieq $Hash) { return }
        Remove-Item $Destination -Force
    }
    $partial = "$Destination.partial"
    try {
        Invoke-WebRequest -Uri $Url -OutFile $partial -TimeoutSec 180 -MaximumRetryCount 3 -RetryIntervalSec 5
        if ((Get-FileHash $partial -Algorithm SHA256).Hash -ine $Hash) { throw "SHA256 mismatch: $Url" }
        Move-Item $partial $Destination -Force
    } finally { if (Test-Path $partial) { Remove-Item $partial -Force } }
}
Push-Location $Repository
try {
    $metadata = cargo metadata --format-version 1 --no-deps --locked --offline | ConvertFrom-Json
    if ($LASTEXITCODE -ne 0) { throw 'Cannot read Cargo package version.' }
    $version = @($metadata.packages | Where-Object name -eq 'flursys')[0].version
    if ($env:GITHUB_REF_TYPE -eq 'tag' -and $env:GITHUB_REF_NAME -ne "v$version") { throw 'Git tag and Cargo package version must match.' }
    $coreVersion = [version]($version -replace '[-+].*$', '')
    $fileVersion = "$($coreVersion.Major).$($coreVersion.Minor).$($coreVersion.Build).0"
    $label = "v$version"
    if ($env:GITHUB_REF_TYPE -ne 'tag' -and $env:GITHUB_SHA) { $label += '-' + $env:GITHUB_SHA.Substring(0, 8) }
    foreach ($name in 'flursys.exe', 'flursys-gui.exe', 'flursys-compressible.exe') {
        if (-not (Test-Path (Join-Path $BuildDirectory $name))) { throw "Missing built executable: $name" }
    }
    $spec = Get-Content (Join-Path $PSScriptRoot 'gmsh.json') -Raw | ConvertFrom-Json
    New-Item -ItemType Directory -Force $CacheDirectory, $OutputDirectory | Out-Null
    $binaryArchive = Join-Path $CacheDirectory "gmsh-$($spec.version)-Windows64.zip"
    $sourceArchive = Join-Path $CacheDirectory "gmsh-$($spec.version)-source.tgz"
    Receive-VerifiedFile $spec.archive_url $spec.archive_sha256 $binaryArchive
    Receive-VerifiedFile $spec.source_url $spec.source_sha256 $sourceArchive
    $temporaryRoot = if ($env:RUNNER_TEMP) { $env:RUNNER_TEMP } else { [IO.Path]::GetTempPath() }
    $work = Join-Path $temporaryRoot ('flursys-package-' + [Guid]::NewGuid().ToString('N'))
    $stage = Join-Path $work 'FLURSYS'
    New-Item -ItemType Directory -Force $stage | Out-Null
    foreach ($name in 'flursys.exe', 'flursys-gui.exe', 'flursys-compressible.exe') { Copy-Item (Join-Path $BuildDirectory $name) $stage }
    foreach ($name in 'LICENSE', 'LICENSE-APACHE', 'THIRD_PARTY_NOTICES.txt') { Copy-Item (Join-Path $Repository $name) $stage }
    Copy-Item (Join-Path $PSScriptRoot 'BUNDLED-TOOLS.txt') $stage
    $licenses = Join-Path $stage 'licenses/solidcraft'
    New-Item -ItemType Directory -Force $licenses | Out-Null
    foreach ($name in 'LICENSE-MIT', 'LICENSE-APACHE', 'NOTICE', 'THIRD-PARTY-LICENSES.txt') { Copy-Item (Join-Path $Repository "vendor/cad-workbench/$name") $licenses }
    $examples = Join-Path $stage 'examples'
    New-Item -ItemType Directory -Force $examples | Out-Null
    Copy-Item (Join-Path $Repository 'examples/cases') $examples -Recurse
    Copy-Item (Join-Path $Repository 'examples/case.schema.json') $examples
    $extracted = Join-Path $work 'gmsh-archive'
    Expand-Archive $binaryArchive $extracted
    $upstream = Join-Path $extracted "gmsh-$($spec.version)-Windows64"
    if (-not (Test-Path (Join-Path $upstream $spec.executable))) { throw 'Official Gmsh archive has an unexpected layout.' }
    Copy-Item $upstream (Join-Path $stage 'gmsh') -Recurse
    $source = Join-Path $stage 'gmsh/source'
    New-Item -ItemType Directory -Force $source | Out-Null
    Copy-Item $sourceArchive $source
    & (Join-Path $env:SystemRoot 'System32/tar.exe') -xzf $sourceArchive -C $work "gmsh-$($spec.version)-source/LICENSE.txt"
    if ($LASTEXITCODE -ne 0) { throw 'Cannot extract the bundled Gmsh license.' }
    Copy-Item (Join-Path $work "gmsh-$($spec.version)-source/LICENSE.txt") (Join-Path $stage 'gmsh/LICENSE-Gmsh.txt')
    Copy-Item (Join-Path $PSScriptRoot 'gmsh.json') (Join-Path $stage 'gmsh/bundle.json')
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
    $runtime = @(& $vswhere -latest -products '*' -find 'VC\Redist\MSVC\**\x64\Microsoft.VC*.CRT\vcruntime140.dll') | Select-Object -Last 1
    if (-not $runtime) { throw 'Visual C++ x64 redistributable DLLs are unavailable on this build machine.' }
    Get-ChildItem (Split-Path $runtime) -Filter '*.dll' | ForEach-Object {
        Copy-Item $_.FullName $stage
        Copy-Item $_.FullName (Join-Path $stage 'gmsh')
    }
    $compiler = @(
        (Join-Path ${env:ProgramFiles(x86)} 'Inno Setup 6/ISCC.exe'),
        (Join-Path $env:ProgramFiles 'Inno Setup 6/ISCC.exe')
    ) | Where-Object { Test-Path $_ } | Select-Object -First 1
    if (-not $compiler) { throw 'Inno Setup 6 is required to create the installer.' }
    $base = "flursys-$label-windows-x86_64"
    & $compiler "/DAppVersion=$version" "/DFileVersion=$fileVersion" "/DStageDir=$stage" "/DOutputDir=$OutputDirectory" "/DOutputName=$base-setup" (Join-Path $PSScriptRoot 'flursys.iss')
    if ($LASTEXITCODE -ne 0) { throw 'Inno Setup compilation failed.' }
    $installer = Join-Path $OutputDirectory "$base-setup.exe"
    $portable = Join-Path $OutputDirectory "$base-portable.zip"
    Compress-Archive -Path (Join-Path $stage '*') -DestinationPath $portable -Force
    @($installer, $portable) | ForEach-Object {
        '{0}  {1}' -f (Get-FileHash $_ -Algorithm SHA256).Hash.ToLowerInvariant(), (Split-Path $_ -Leaf)
    } | Set-Content (Join-Path $OutputDirectory 'flursys-windows-x86_64-checksums.txt') -Encoding utf8NoBOM
    if ($env:GITHUB_OUTPUT) {
        "installer=$installer" | Out-File $env:GITHUB_OUTPUT -Append -Encoding utf8
        "portable=$portable" | Out-File $env:GITHUB_OUTPUT -Append -Encoding utf8
    }
} finally {
    Pop-Location
    if ($work -and (Test-Path $work)) { Remove-Item $work -Recurse -Force }
}
