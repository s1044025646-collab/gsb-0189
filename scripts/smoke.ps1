$env:PORT = "8799"
$p = Start-Process node -ArgumentList "dist/server.js" -PassThru -WindowStyle Hidden
Start-Sleep 2
try {
  Invoke-RestMethod http://localhost:8799/api/health | ConvertTo-Json -Compress
  $body = '{"point":{"x":120,"y":-80,"z":900},"intrinsics":{"fx":800,"fy":800,"cx":320,"cy":240,"k1":-0.28,"k2":0.12}}'
  $pr = Invoke-RestMethod -Method Post -Uri http://localhost:8799/api/project -ContentType "application/json" -Body $body
  "project u,v = $([math]::Round($pr.u,3)), $([math]::Round($pr.v,3))"
  $ub = '{"u":425.908,"v":169.395,"depth":900,"intrinsics":{"fx":800,"fy":800,"cx":320,"cy":240,"k1":-0.28,"k2":0.12}}'
  $ur = Invoke-RestMethod -Method Post -Uri http://localhost:8799/api/unproject -ContentType "application/json" -Body $ub
  ("unproject ok=" + $ur.ok + " X=" + [math]::Round($ur.point.x, 3) + " Y=" + [math]::Round($ur.point.y, 3))
} finally {
  Stop-Process $p.Id
}
