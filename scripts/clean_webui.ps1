$file = "f:\ninjaidf\xiaozhi-esp32-ninja2\xiaozhi-esp32-2.0.5\main\boards\otto-ninja\webserver.c"
$c = [System.IO.File]::ReadAllText($file, [System.Text.Encoding]::UTF8)

# 1. Rename tab button
$c = $c.Replace(
    '"<button class=\"tab active\" onclick=\"switchTab(1)\">' + [char]0x1F3AE + ' Di' + [char]0x1EC1 + 'u Khi' + [char]0x1EC3 + 'n</button>"',
    '"<button class=\"tab active\" onclick=\"switchTab(1)\">' + [char]0x1F4A1 + ' LED</button>"'
)
Write-Host "Step 1 (rename tab): done"

# 2. Remove tab1 inner HTML from instructions banner through export/import closing
#    Start marker: instructions div opening
#    End marker: jsonStatus closing + its parent closing
$startMark = '"<div style=\"background:#1a252f;padding:8px;border-radius:6px;margin:5px 0 10px 0;font-size:11px;line-height:1.5;\">"'
$endMark   = '"<div id=\"jsonStatus\" style=\"padding:6px;font-size:0.75em;color:#1abc9c;margin-top:6px\"></div>"' + "`r`n" + '"</div>"'
$endMark2  = '"<div id=\"jsonStatus\" style=\"padding:6px;font-size:0.75em;color:#1abc9c;margin-top:6px\"></div>"' + "`n" + '"</div>"'

$si = $c.IndexOf($startMark)
$ei = $c.IndexOf($endMark)
if ($ei -lt 0) { $ei = $c.IndexOf($endMark2) }

if ($si -ge 0 -and $ei -ge 0) {
    $removeEnd = $ei + [Math]::Max($endMark.Length, $endMark2.Length)
    # find actual length used
    $usedEnd = $endMark
    if ($c.IndexOf($endMark) -lt 0) { $usedEnd = $endMark2 }
    $removeEnd = $ei + $usedEnd.Length
    $c = $c.Substring(0, $si) + $c.Substring($removeEnd)
    Write-Host "Step 2 (remove tab1 controls HTML): done (removed $($removeEnd-$si) chars)"
} else {
    Write-Host "Step 2: markers not found! si=$si ei=$ei"
    Write-Host "Searching for start..."
    $idx = $c.IndexOf("background:#1a252f")
    Write-Host "  'background:#1a252f' at: $idx"
    $idx2 = $c.IndexOf("jsonStatus")
    Write-Host "  'jsonStatus' at: $idx2"
}

# 3. Remove JS control section: from joystick vars through updateSlotInfo()
$jsStart = '"let jx=0,jy=0,bA=0,bB=0,bX=0,bY=0;"'
$jsEnd   = '"updateSlotInfo();"'
$jsi = $c.IndexOf($jsStart)
$jei = $c.IndexOf($jsEnd)
if ($jsi -ge 0 -and $jei -ge 0) {
    $jeiEnd = $jei + $jsEnd.Length
    # Replace with just fb function + testfoot handlers
    $replacement = '"function fb(id){const b=document.getElementById(id);b.classList.add(''clicked'');setTimeout(()=>b.classList.remove(''clicked''),150);}"' + "`r`n" +
        '"document.getElementById(''btnTestLF'').onclick=()=>{fb(''btnTestLF'');fetch(''/testfoot?foot=left'');};"' + "`r`n" +
        '"document.getElementById(''btnTestRF'').onclick=()=>{fb(''btnTestRF'');fetch(''/testfoot?foot=right'');};"' + "`r`n" +
        '"document.getElementById(''btnTestBoth'').onclick=()=>{fb(''btnTestBoth'');fetch(''/testfoot?foot=both'');};"' + "`r`n" +
        '"document.getElementById(''btnStopFoot'').onclick=()=>{fb(''btnStopFoot'');fetch(''/testfoot?foot=stop'');};"'
    $c = $c.Substring(0, $jsi) + $replacement + $c.Substring($jeiEnd)
    Write-Host "Step 3 (remove JS control block): done"
} else {
    Write-Host "Step 3: markers not found! jsi=$jsi jei=$jei"
}

# 4. Fix btnApply and btnSaveTop: remove turnComboParams references
$c = $c -replace "const turnComboParams='&tls='\+document\.getElementById\('turnLeftSpeed'\)\.value\+'&trs='\+document\.getElementById\('turnRightSpeed'\)\.value\+'&clf='\+document\.getElementById\('comboLfSpeed'\)\.value\+'&crf='\+document\.getElementById\('comboRfSpeed'\)\.value;`r?`n", ""
$c = $c -replace "fetch\('/calibrate\?'\+params\+turnComboParams\+batteryAlert\)", "fetch('/calibrate?'+params+batteryAlert)"
Write-Host "Step 4 (fix btnApply/btnSaveTop): done"

# 5. Remove getCal references to removed sliders
$c = $c -replace '"if\(d\.tls\)\{[^"]+\}"' + "`r?`n", ""
$c = $c -replace '"if\(d\.trs\)\{[^"]+\}"' + "`r?`n", ""
$c = $c -replace '"if\(d\.clf\)\{[^"]+\}"' + "`r?`n", ""
$c = $c -replace '"if\(d\.crf\)\{[^"]+\}"' + "`r?`n", ""
Write-Host "Step 5 (fix getCal): done"

[System.IO.File]::WriteAllText($file, $c, [System.Text.Encoding]::UTF8)
Write-Host "All changes written to file"
