@echo on
set DIR=%3
set DST=%2\..\GameData\KramaxAutoPilot\Plugins

if not exist %DST% mkdir %DST%
if %1%==Release (
	copy %DIR%KramaxAutoPilot.* %DST%
)
