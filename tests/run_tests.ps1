$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$outputDir = Join-Path $projectRoot 'build\host-tests'
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$compiler = (Get-Command 'g++' -ErrorAction Stop).Source
$commonArgs = @(
    '-std=c++17', '-Wall', '-Wextra', '-Wpedantic',
    ('-I' + (Join-Path $PSScriptRoot 'hal_stubs')),
    ('-I' + (Join-Path $projectRoot 'C_board_src')),
    (Join-Path $PSScriptRoot 'control_framework_test.cpp'),
    (Join-Path $projectRoot 'C_board_src\ControlTask.cpp'),
    (Join-Path $projectRoot 'C_board_src\Gm6020.cpp'),
    (Join-Path $projectRoot 'C_board_src\AssessmentController.cpp'),
    (Join-Path $projectRoot 'C_board_src\Pid.cpp')
)
$cases = @(
    @{ Name = 'default-can1'; Flags = @() },
    @{ Name = 'derivative-enabled'; Flags = @('-DCONTROL_ASSESSMENT_SPEED_KD=0.5f') },
    @{ Name = 'fixed-speed-curves'; Flags = @('-DCONTROL_ASSESSMENT_FIXED_MODE=4','-DCONTROL_ASSESSMENT_SPEED_AMPLITUDE=8.0f','-DCONTROL_ASSESSMENT_SPEED_KP=220.0f','-DCONTROL_ASSESSMENT_SPEED_KI=100.0f') },
    @{ Name = 'fixed-position-curves'; Flags = @('-DCONTROL_ASSESSMENT_FIXED_MODE=5','-DCONTROL_ASSESSMENT_SPEED_AMPLITUDE=8.0f','-DCONTROL_ASSESSMENT_SPEED_KP=220.0f','-DCONTROL_ASSESSMENT_SPEED_KI=100.0f') },
    @{ Name = 'angle-ff-disabled'; Flags = @('-DCONTROL_ASSESSMENT_ANGLE_FF_ENABLED=0') },
    @{ Name = 'voltage-id1'; Flags = @('-DCONTROL_COMMAND_CURRENT=0') },
    @{ Name = 'voltage-id5'; Flags = @('-DCONTROL_COMMAND_CURRENT=0', '-DCONTROL_MOTOR_ID=5') },
    @{ Name = 'transmit-disabled'; Flags = @('-DCONTROL_CAN_TX_ENABLED=0') },
    @{ Name = 'exact-id-filter'; Flags = @('-DCONTROL_CAN_DIAGNOSTIC_RX=0') },
    @{ Name = 'default-can2'; Flags = @('-DCONTROL_CAN_CHANNEL=2') },
    @{ Name = 'test-pi'; Flags = @('-DCONTROL_SPEED_KP=4.0f', '-DCONTROL_SPEED_KI=1.0f', '-DCONTROL_SPEED_OUTPUT_LIMIT=5.0f') },
    @{ Name = 'calculation-disabled'; Flags = @('-DCONTROL_SPEED_CALC_ENABLED=0', '-DCONTROL_SPEED_KP=4.0f') },
    @{ Name = 'tx-can1-id1'; Flags = @('-DCONTROL_CAN_TX_ENABLED=1', '-DCONTROL_SPEED_KP=4.0f', '-DCONTROL_SPEED_KI=1.0f', '-DCONTROL_SPEED_OUTPUT_LIMIT=5.0f') },
    @{ Name = 'tx-can2-id5'; Flags = @('-DCONTROL_CAN_TX_ENABLED=1', '-DCONTROL_CAN_CHANNEL=2', '-DCONTROL_MOTOR_ID=5') },
    @{ Name = 'tx-id4'; Flags = @('-DCONTROL_CAN_TX_ENABLED=1', '-DCONTROL_MOTOR_ID=4') },
    @{ Name = 'tx-id7'; Flags = @('-DCONTROL_CAN_TX_ENABLED=1', '-DCONTROL_MOTOR_ID=7') },
    @{ Name = 'tx-calculation-disabled'; Flags = @('-DCONTROL_CAN_TX_ENABLED=1', '-DCONTROL_SPEED_CALC_ENABLED=0') }
)
foreach ($case in $cases) {
    $testExe = Join-Path $outputDir ($case.Name + '.exe')
    $compilerArgs = $commonArgs + $case.Flags + @('-o', $testExe)
    & $compiler @compilerArgs
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $($case.Name)" }
    Write-Output "Test: $($case.Name)"
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $($case.Name)" }
}

$inputTest = Join-Path $outputDir 'input-control.exe'
& $compiler '-std=c++17' '-Wall' '-Wextra' '-Wpedantic' `
    ('-I' + (Join-Path $PSScriptRoot 'hal_stubs')) `
    ('-I' + (Join-Path $projectRoot 'C_board_src')) `
    (Join-Path $PSScriptRoot 'input_control_test.cpp') `
    (Join-Path $projectRoot 'C_board_src/SerialCommand.cpp') `
    (Join-Path $projectRoot 'C_board_src/KeyControl.cpp') '-o' $inputTest
if ($LASTEXITCODE -ne 0) { throw 'Compile failed: input-control' }
& $inputTest
if ($LASTEXITCODE -ne 0) { throw 'Test failed: input-control' }
