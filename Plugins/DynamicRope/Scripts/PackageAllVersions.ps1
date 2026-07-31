# PackageAllVersions.ps1 — build and zip the DynamicRope plugin for every supported engine version.
#
# For each requested version this script:
#   1. Stages the plugin source (5.7 builds straight from the Perforce workspace; other versions
#      are mirrored into their staging project copy first — 5.5 keeps its own separately-authored
#      Content and receives everything else).
#   2. Runs that engine's `RunUAT BuildPlugin` into C:\DynamicRopeVersions\<ver>\DynamicRope.
#   3. Zips the package for Fab upload into C:\DynamicRopeVersions\DynamicRope_<ver>.zip:
#      package output minus Binaries / Intermediate / FabURL*, plus the plugin's /Config folder,
#      under a single `DynamicRope/` root — matching the manual zips made on 2026-07-31.
#
# Versions build SEQUENTIALLY on purpose: two concurrent UBT instances collide on the shared
# %LOCALAPPDATA%\UnrealBuildTool log and on the UBT mutex.
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File PackageAllVersions.ps1              # all versions
#   powershell -ExecutionPolicy Bypass -File PackageAllVersions.ps1 -Versions 5.8
#   powershell -ExecutionPolicy Bypass -File PackageAllVersions.ps1 -SkipZip

param(
	[string[]] $Versions = @('5.5', '5.6', '5.7', '5.8'),
	[switch] $SkipZip
)

$ErrorActionPreference = 'Stop'

$Workspace   = 'C:\Users\Shimwoojin\Perforce\Shimwoojin_Main\DynamicRopeProject'
$SourcePlugin = Join-Path $Workspace 'Plugins\DynamicRope'
$OutRoot     = 'C:\DynamicRopeVersions'
$EngineRoot  = 'C:\Program Files\Epic Games'

# Per-version staging: which plugin folder is handed to BuildPlugin, and whether Content is
# mirrored from the workspace (5.5 authors its own Content for older-version asset stamps).
$Stages = @{
	'5.5' = @{ Plugin = 'C:\MyProjects\Plug55\Plugins\DynamicRope';   SyncContent = $false }
	'5.6' = @{ Plugin = 'C:\MyProjects\View5_6\Plugins\DynamicRope';  SyncContent = $true }
	'5.7' = @{ Plugin = $SourcePlugin;                                 SyncContent = $false } # builds in place
	'5.8' = @{ Plugin = 'C:\MyProjects\Plug58\Plugins\DynamicRope';   SyncContent = $true }
}

# Robocopy exit codes 0-7 mean success (files copied / already in sync); 8+ is failure.
function Invoke-Mirror([string] $From, [string] $To) {
	robocopy $From $To /MIR /A-:R /NJH /NJS /NDL /NP /NFL | Out-Null
	if ($LASTEXITCODE -ge 8) { throw "robocopy failed ($LASTEXITCODE): $From -> $To" }
}

function Stage-Plugin([string] $Ver) {
	$stage = $Stages[$Ver]
	if ($stage.Plugin -eq $SourcePlugin) { return }   # 5.7: nothing to stage

	$dirs = @('Source', 'Shaders', 'Config', 'Resources', 'Docs')
	if ($stage.SyncContent) { $dirs += 'Content' }
	foreach ($d in $dirs) {
		Invoke-Mirror (Join-Path $SourcePlugin $d) (Join-Path $stage.Plugin $d)
	}
	foreach ($f in @('DynamicRope.uplugin', 'README.md')) {
		Copy-Item (Join-Path $SourcePlugin $f) $stage.Plugin -Force
		$dest = Join-Path $stage.Plugin $f
		Set-ItemProperty $dest -Name IsReadOnly -Value $false
		(Get-Item $dest).LastWriteTime = Get-Date
	}
}

function Build-Version([string] $Ver) {
	$uat = Join-Path $EngineRoot "UE_$Ver\Engine\Build\BatchFiles\RunUAT.bat"
	if (-not (Test-Path $uat)) { throw "engine not found: $uat" }
	$uplugin = Join-Path $Stages[$Ver].Plugin 'DynamicRope.uplugin'
	$pkgDir  = Join-Path $OutRoot "$Ver\DynamicRope"
	$log     = Join-Path $OutRoot "logs\BuildPlugin_$Ver.log"
	New-Item -ItemType Directory -Force (Split-Path $log) | Out-Null

	& $uat BuildPlugin -Plugin="$uplugin" -Package="$pkgDir" -CreateSubFolder -nocompile -nocompileuat *> $log
	if ($LASTEXITCODE -ne 0) { throw "BuildPlugin $Ver failed (exit $LASTEXITCODE) - see $log" }

	# The package must carry the readme (FilterPlugin.ini /README.md rule); fail loudly if not.
	if (-not (Test-Path (Join-Path $pkgDir 'README.md'))) { throw "$Ver package is missing README.md" }
	return $pkgDir
}

function Zip-Version([string] $Ver, [string] $PkgDir) {
	$stageRoot = Join-Path $OutRoot '_zipstage'
	$stage     = Join-Path $stageRoot 'DynamicRope'
	if (Test-Path $stageRoot) { Remove-Item $stageRoot -Recurse -Force }

	# Fab source upload: no Binaries, no Intermediate, no FabURL leftovers.
	robocopy $PkgDir $stage /MIR /XD Binaries Intermediate /XF FabURL* /NJH /NJS /NDL /NP /NFL | Out-Null
	if ($LASTEXITCODE -ge 8) { throw "zip staging failed for $Ver" }
	# The package itself does not carry /Config; the zip does (FilterPlugin.ini documents the filter).
	Invoke-Mirror (Join-Path $Stages[$Ver].Plugin 'Config') (Join-Path $stage 'Config')

	$zip = Join-Path $OutRoot "DynamicRope_$Ver.zip"
	if (Test-Path $zip) { Remove-Item $zip -Force }
	Compress-Archive -Path $stage -DestinationPath $zip
	Remove-Item $stageRoot -Recurse -Force
	return $zip
}

$results = @()
foreach ($ver in $Versions) {
	if (-not $Stages.ContainsKey($ver)) { throw "unknown version '$ver' (expected 5.5/5.6/5.7/5.8)" }
	Write-Host "==== $ver : staging ===="
	try {
		Stage-Plugin $ver
		Write-Host "==== $ver : BuildPlugin (UE_$ver) ===="
		$pkg = Build-Version $ver
		$zip = $null
		if (-not $SkipZip) {
			Write-Host "==== $ver : zip ===="
			$zip = Zip-Version $ver $pkg
		}
		$results += [pscustomobject]@{ Version = $ver; Result = 'OK'; Package = $pkg; Zip = $zip }
	}
	catch {
		$results += [pscustomobject]@{ Version = $ver; Result = "FAILED: $_"; Package = $null; Zip = $null }
	}
}

Write-Host ''
$results | Format-Table -AutoSize -Wrap
if ($results | Where-Object { $_.Result -ne 'OK' }) { exit 1 }
Write-Host 'All requested versions packaged.'
