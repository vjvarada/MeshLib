$SOURCE_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\MeshLib\standalone\source"
$TEMP_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\MeshLib\standalone\temp_bindings"
$COMBINED_HEADER = "$TEMP_DIR\all_headers.h"

$lines = @()
$lines += "// Combined header for mrbind - includes all MRMesh headers"
$lines += "#pragma once"
$lines += "// CRITICAL: Fix offsetof BEFORE any system headers"
$lines += "#include <stddef.h>"
$lines += "#undef offsetof"
$lines += "#define offsetof(s,m) __builtin_offsetof(s,m)"
$lines += "#include <iostream>"
$lines += ""

# Headers that MUST be included early (before MRMeshFwd.h which has forward declarations)
# These headers define types that are forward-declared in MRMeshFwd.h
$earlyHeaders = @(
    "MRUnorientedTriangle.h",
    "MRTriangleIntersection.h",
    "MRClosestPointInTriangle.h"
)

$lines += "// Early includes - define types before forward declarations in MRMeshFwd.h"
foreach ($h in $earlyHeaders) {
    $fullPath = "$SOURCE_DIR\MRMesh\$h"
    if (Test-Path $fullPath) {
        $lines += ('#include "MRMesh/' + $h + '"')
    }
}
$lines += ""

$headers = Get-ChildItem "$SOURCE_DIR\MRMesh\MR*.h" | Where-Object { 
    $_.Name -notmatch '^MR.*GTest|^MR.*Viewer|^MRImGui|^MRGL|^MRCuda' -and
    $_.Name -notin $earlyHeaders
} | Sort-Object Name

foreach ($h in $headers) {
    $lines += ('#include "MRMesh/' + $h.Name + '"')
}

Set-Content -Path $COMBINED_HEADER -Value ($lines -join "
")
Write-Host "Generated $COMBINED_HEADER with $($headers.Count) includes"
