[CmdletBinding()]
param([Parameter(Mandatory)][string]$Installer)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$root = Join-Path $env:RUNNER_TEMP ('flursys-install-smoke-' + [Guid]::NewGuid().ToString('N'))
$installed = Join-Path $root 'FLURSYS installed with spaces'
$workspace = Join-Path $root 'user workspace'
$previousPath = $env:PATH
try {
    $setup = Start-Process -FilePath $Installer -ArgumentList @('/VERYSILENT', '/SUPPRESSMSGBOXES', '/NORESTART', "/DIR=`"$installed`"") -Wait -PassThru
    if ($setup.ExitCode -ne 0) { throw "Installer failed: $($setup.ExitCode)" }
    foreach ($name in 'flursys.exe', 'flursys-gui.exe', 'gmsh/gmsh.exe') {
        if (-not (Test-Path (Join-Path $installed $name))) { throw "Installed file missing: $name" }
    }
    foreach ($binary in @(@{ Name = 'flursys-gui.exe'; Subsystem = 2 }, @{ Name = 'flursys.exe'; Subsystem = 3 })) {
        $bytes = [IO.File]::ReadAllBytes((Join-Path $installed $binary.Name))
        if ($bytes.Length -lt 64) { throw "Invalid executable: $($binary.Name)" }
        $pe = [BitConverter]::ToInt32($bytes, 60)
        if ($pe -lt 0 -or $pe -gt ($bytes.Length - 94) -or [BitConverter]::ToUInt32($bytes, $pe) -ne 0x4550) {
            throw "Invalid PE header: $($binary.Name)"
        }
        $subsystem = [BitConverter]::ToUInt16($bytes, $pe + 92)
        if ($subsystem -ne $binary.Subsystem) { throw "Wrong Windows subsystem for $($binary.Name): $subsystem" }
    }
    & (Join-Path $installed 'flursys.exe') --version
    if ($LASTEXITCODE -ne 0) { throw 'Installed CLI cannot start.' }
    & (Join-Path $installed 'gmsh/gmsh.exe') --version
    if ($LASTEXITCODE -ne 0) { throw 'Bundled Gmsh cannot start.' }
    & (Join-Path $installed 'flursys.exe') examples create laminar-channel $workspace --preset coarse --json
    if ($LASTEXITCODE -ne 0) { throw 'Installed case library failed.' }
    $env:PATH = "$env:SystemRoot\System32;$env:SystemRoot"
    & (Join-Path $installed 'flursys.exe') project mesh $workspace --json
    if ($LASTEXITCODE -ne 0) { throw 'Meshing failed without Gmsh on PATH.' }
    $env:PATH = $previousPath
    $uninstall = Start-Process -FilePath (Join-Path $installed 'unins000.exe') -ArgumentList @('/VERYSILENT', '/SUPPRESSMSGBOXES', '/NORESTART') -Wait -PassThru
    if ($uninstall.ExitCode -ne 0) { throw 'Silent uninstall failed.' }
    if (Test-Path (Join-Path $installed 'flursys-gui.exe')) { throw 'Uninstall left the application executable behind.' }
    if (-not (Test-Path (Join-Path $workspace 'project.json'))) { throw 'Uninstall affected an external workspace.' }
} finally {
    $env:PATH = $previousPath
    if (Test-Path $root) { Remove-Item $root -Recurse -Force }
}
