<#
.SYNOPSIS
  给「含中文但缺 BOM 的 UTF-8 源文件」补上 UTF-8 BOM。

.DESCRIPTION
  本工程 Keil(AC5) 与 EIDE(GCC) 双工具链并存, 编码要求:
    - Keil AC5 默认按本地编码(cp936/GBK)解析源文件 → GBK 文件没问题;
      但对「UTF-8 无 BOM」的文件, 会按 GBK 成对读取字节, 可能把后面的 " 或 )
      当成汉字的一部分吃掉 → 报错:
        #18: expected a ")"  /  #274: improperly terminated macro invocation
    - 实测: 只要文件带 UTF-8 BOM(EF BB BF), AC5 就按 UTF-8 正确解析;
      GCC 同样支持 BOM。
  所以规则: 含非 ASCII 的文件 → 要么 GBK, 要么 UTF-8+BOM; 不许"UTF-8 无 BOM"。
  本脚本只处理最后那种(补 BOM), 纯 ASCII / GBK 文件一律不动。

.PARAMETER Report
  只报告, 不修改文件。

.EXAMPLE
  powershell -NoProfile -ExecutionPolicy Bypass -File tools\fix-src-encoding.ps1
  powershell -NoProfile -ExecutionPolicy Bypass -File tools\fix-src-encoding.ps1 -Report
#>
param(
    [string[]]$Paths = @('Core', 'Drivers'),
    [switch]$Report
)

$root = Split-Path -Parent (Split-Path -Parent $PSCommandPath)
if (-not (Test-Path (Join-Path $root 'Core'))) {
    Write-Error "找不到工程根目录(脚本应放在 <工程根>\tools\ 下): $root"
    exit 1
}

$utf8Bom = New-Object System.Text.UTF8Encoding($true)
$withBom = New-Object System.Collections.ArrayList
$gbkFiles = New-Object System.Collections.ArrayList
$fixed = New-Object System.Collections.ArrayList

foreach ($p in $Paths) {
    $dir = Join-Path $root $p
    if (-not (Test-Path $dir)) { continue }

    foreach ($file in (Get-ChildItem -Path $dir -Recurse -Include *.c,*.h -File)) {
        $bytes = [IO.File]::ReadAllBytes($file.FullName)
        if ($bytes.Length -lt 3) { continue }

        $hasBom = ($bytes[0] -eq 0xEF) -and ($bytes[1] -eq 0xBB) -and ($bytes[2] -eq 0xBF)
        $text = [Text.Encoding]::UTF8.GetString($bytes)
        $isUtf8 = ([Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($text)) -eq `
                   [Convert]::ToBase64String($bytes))
        $hasNonAscii = ($text -match '[^\x00-\x7F]')

        if (-not $hasNonAscii) { continue }          # 纯 ASCII: 任何编码都一样, 不用管
        if ($hasBom) { [void]$withBom.Add($file.FullName); continue }
        if (-not $isUtf8) { [void]$gbkFiles.Add($file.FullName); continue }  # GBK: Keil 能正确解析, 不动

        if (-not $Report) {
            [IO.File]::WriteAllText($file.FullName, $text, $utf8Bom)
        }
        [void]$fixed.Add($file.FullName)
    }
}

$rel = { param($p) $p.Replace($root + '\', '') }

if ($Report) { $fixedLabel = 'UTF8 无BOM(需补)' } else { $fixedLabel = '已补 UTF8 BOM' }

Write-Host ("[编码检查] 根目录: {0}" -f $root)
Write-Host ("  已是 UTF8+BOM   : {0} 个" -f $withBom.Count)
Write-Host ("  GBK(不动)       : {0} 个" -f $gbkFiles.Count)
Write-Host ("  {0}: {1} 个" -f $fixedLabel, $fixed.Count)

if ($fixed.Count -gt 0) {
    Write-Host "  --- 本次处理 ---"
    foreach ($f in $fixed) { Write-Host ("    " + (& $rel $f)) }
}
if ($Report -and $gbkFiles.Count -gt 0) {
    Write-Host "  --- GBK 文件(Keil 按本地编码解析, 正常) ---"
    foreach ($f in $gbkFiles) { Write-Host ("    " + (& $rel $f)) }
}
