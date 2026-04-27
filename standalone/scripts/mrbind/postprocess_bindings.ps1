# Post-process bindings to remove problematic operator functions
$file = "C:\Users\VijayRaghavVarada\Documents\Github\MeshLib\standalone\temp_bindings\bindings_all.cpp"
$content = Get-Content $file -Raw

# Patterns to comment out - operators in MB_FUNC that cause compilation errors
# These are multi-line patterns starting with MB_FUNC and ending with )
$patterns = @(
    '(?s)MB_FUNC\([^)]*operator&[^)]*TypedBitSet[^)]*\)\s*\n[^)]+\n[^)]+\n\)',
    '(?s)MB_FUNC\([^)]*operator\|[^)]*TypedBitSet[^)]*\)\s*\n[^)]+\n[^)]+\n\)',
    '(?s)MB_FUNC\([^)]*operator\^[^)]*TypedBitSet[^)]*\)\s*\n[^)]+\n[^)]+\n\)',
    '(?s)MB_FUNC\([^)]*operator-[^)]*TypedBitSet[^)]*\)\s*\n[^)]+\n[^)]+\n\)'
)

foreach ($pattern in $patterns) {
    $content = [regex]::Replace($content, $pattern, '/* REMOVED: operator binding */')
}

Set-Content -Path $file -Value $content -NoNewline
Write-Host "Post-processed bindings_all.cpp"
