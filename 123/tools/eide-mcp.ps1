<#
.SYNOPSIS
  EIDE 自动化工具 —— 重载工程 / 构建 / 重新构建 / 清理 / 烧录

.DESCRIPTION
  通过 EIDE 内置 MCP 服务器 (Streamable HTTP, 默认 127.0.0.1:8940) 调用
  eide_reload / eide_build / eide_rebuild / eide_clean / eide_flash 等工具。

  适用场景: 在外部手工修改 .eide/eide.yml 等工程配置后（例如移植新模块、
  添加新源文件），EIDE 会进入 "reload pending" 状态并禁用 Build 按钮。
  用本脚本可自动完成 "重载工程 + 构建"，无需在 EIDE 工程树里手动点 reload。

  前置条件:
    - VS Code 中 EIDE 扩展已加载，且启用了 MCP 服务
      (设置 EIDE.MCP.Server.Enable = true，默认端口 8940)

.PARAMETER Action
  reload  : 仅重载工程
  build   : 仅构建 (增量)
  rebuild : 重载 + 全量重建
  clean   : 清理 build 目录
  flash   : 重载 + 烧录 (ST-Link, 不整片擦除)
  all     : 重载 + 构建 (默认)

.PARAMETER Port
  EIDE MCP 服务器 HTTP 端口 (默认 8940)

.EXAMPLE
  .\tools\eide-mcp.ps1                 # = reload + build
  .\tools\eide-mcp.ps1 -Action rebuild # 重载 + 全量重建
  .\tools\eide-mcp.ps1 -Action flash   # 重载 + 烧录
#>
param(
    [ValidateSet('reload','build','rebuild','clean','flash','all')]
    [string]$Action = 'all',
    [int]$Port = 8940
)

$ErrorActionPreference = 'Stop'

# ---- 定位工程根: 脚本放在 <工程根>/tools 下 ----
$Root = Split-Path -Parent $PSScriptRoot
$EideYml = Join-Path $Root '.eide\eide.yml'
$BaseUrl = "http://127.0.0.1:$Port"

function Write-Step([string]$msg) { Write-Host "[eide-mcp] $msg" -ForegroundColor Cyan }

# ---- 1. 健康检查 ----
try {
    $h = Invoke-RestMethod -Uri "$BaseUrl/health" -TimeoutSec 5
}
catch {
    throw "EIDE MCP 服务不可用 ($BaseUrl): $($_.Exception.Message)`n请确认: VS Code 已打开本工作区且 EIDE 扩展已加载, 并已启用 MCP (设置 EIDE.MCP.Server.Enable)。"
}
if (-not $h.ok) { throw 'EIDE MCP 服务健康检查未通过' }
Write-Step "MCP server ok (version=$($h.version), httpPort=$($h.httpPort))"

# ---- 2. 读取工程 uid ----
if (-not (Test-Path $EideYml)) { throw "找不到工程配置: $EideYml" }
$m = Select-String -Path $EideYml -Pattern '^\s*uid:\s*(\S+)'
if (-not $m) { throw 'eide.yml 中找不到 miscInfo.uid' }
$Uid = $m.Matches[0].Groups[1].Value
Write-Step "project uid = $Uid"

# ---- 3. MCP 客户端 (Streamable HTTP) ----
$script:SessionId = $null
$script:NextId = 100

function Invoke-Mcp {
    param(
        [string]$Method,
        $Params,
        [switch]$Notify
    )
    $req = [ordered]@{ jsonrpc = '2.0'; method = $Method }
    if (-not $Notify) { $req.id = $script:NextId; $script:NextId++ }
    if ($null -ne $Params) { $req.params = $Params }
    $body = $req | ConvertTo-Json -Depth 12

    $headers = @{ Accept = 'application/json, text/event-stream' }
    if ($script:SessionId) { $headers['Mcp-Session-Id'] = $script:SessionId }

    $resp = Invoke-WebRequest -Uri "$BaseUrl/mcp" -Method Post -ContentType 'application/json' `
            -Headers $headers -Body $body -UseBasicParsing -TimeoutSec 900

    if ($resp.Headers['Mcp-Session-Id']) { $script:SessionId = $resp.Headers['Mcp-Session-Id'] }
    if ($Notify) { return $null }

    # 解析 SSE 响应: 收集所有 data: 行, 取最后一条(含最终结果)
    $data = $resp.Content -split "`r?`n" |
            Where-Object { $_ -like 'data:*' } |
            ForEach-Object { ($_ -replace '^data: ?','').Trim() } |
            Where-Object { $_ -ne '' }
    if (-not $data) { throw 'MCP 响应中没有 data 行' }
    $last = $data | Select-Object -Last 1
    return ($last | ConvertFrom-Json)
}

function Invoke-Tool {
    param([string]$Name, $ToolArgs)
    Write-Step "调用工具 $Name ..."
    $resp = Invoke-Mcp -Method 'tools/call' -Params @{ name = $Name; arguments = $ToolArgs }
    if ($resp.error) { throw "MCP 返回错误: $($resp.error.message)" }
    $res = $resp.result
    $text = ($res.content | Where-Object { $_.type -eq 'text' } | ForEach-Object { $_.text }) -join "`n"
    if ($res.isError) {
        Write-Host "[eide-mcp] $Name 执行失败: $text" -ForegroundColor Red
        exit 1
    }
    Write-Host "[eide-mcp] $Name 完成: $text" -ForegroundColor Green
}

# ---- 4. MCP 握手 ----
Write-Step 'MCP initialize ...'
$null = Invoke-Mcp -Method 'initialize' -Params @{
    protocolVersion = '2025-11-25'
    capabilities    = @{}
    clientInfo      = @{ name = 'eide-mcp-ps'; version = '1.0' }
}
$null = Invoke-Mcp -Method 'notifications/initialized' -Notify

# ---- 5. 按 Action 执行 ----
switch ($Action) {
    'reload'  { Invoke-Tool 'eide_reload' @{ uid = $Uid } }
    'build'   { Invoke-Tool 'eide_build'  @{ uid = $Uid } }
    'rebuild' { Invoke-Tool 'eide_reload' @{ uid = $Uid }; Invoke-Tool 'eide_rebuild' @{ uid = $Uid } }
    'clean'   { Invoke-Tool 'eide_clean'  @{ uid = $Uid } }
    'flash'   { Invoke-Tool 'eide_reload' @{ uid = $Uid }; Invoke-Tool 'eide_flash' @{ uid = $Uid; eraseAll = $false } }
    'all'     { Invoke-Tool 'eide_reload' @{ uid = $Uid }; Invoke-Tool 'eide_build'  @{ uid = $Uid } }
}

Write-Step 'Done.'
