# Install the isolated offline CV environment from the workspace root.
$ErrorActionPreference = 'Stop'
$workspace = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
Push-Location $workspace
try {
    if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
        throw 'Install uv from https://docs.astral.sh/uv/getting-started/installation/ first.'
    }
    uv python install 3.12
    if ($LASTEXITCODE -ne 0) { throw 'Python installation failed' }
    if (-not (Test-Path -LiteralPath '.venv-perception/Scripts/python.exe')) {
        uv venv --python 3.12 .venv-perception
        if ($LASTEXITCODE -ne 0) { throw 'Environment creation failed' }
    }
    uv pip install --python .venv-perception/Scripts/python.exe -r src/opponent_perception/requirements-offline.txt
    if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed' }
    uv pip install --python .venv-perception/Scripts/python.exe --no-deps -e src/opponent_perception
    if ($LASTEXITCODE -ne 0) { throw 'Package installation failed' }
    & .venv-perception/Scripts/python.exe -m pytest src/opponent_perception/test -m 'not ros' -q
    if ($LASTEXITCODE -ne 0) { throw 'Offline tests failed' }
} finally {
    Pop-Location
}
