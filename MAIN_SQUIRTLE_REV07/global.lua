--[[
==================================================================
  ARQUIVO: global.lua
  DESCRIÇÃO: Define variáveis globais e funções auxiliares que
             serão compartilhadas entre a thread principal (src0.lua)
             e a thread de monitoramento (src2.lua).
==================================================================
--]]
print("Inicializando definições globais...")
id = 0
currentState = 0
data_x = 0
data_y = 0
data_z = 0
att = 0

-- Definição dos Estados da Máquina de Estados (Fluxo Principal)
STATE_INIT = "INIT"
STATE_IDLE = "IDLE"
STATE_VALIDATE = "VALIDATE_POSITION"
STATE_MANAGE_SCREW_INVENTORY = "MANAGE_SCREW_INVENTORY"
STATE_PICK_SCREW = "PICK_SCREW"
STATE_APPROACH_POINT = "APPROACH_POINT"
STATE_EXECUTE_SCREWING = "EXECUTE_SCREWING"
STATE_RESET = "RESET_REGISTER"
STATE_RETRACT = "RETRACT"
STATE_CLOSE = "CLOSE_MODBUS"
STATE_ERROR = "ERROR"

-- Definição dos Novos Estados da Máquina de Estados (Fluxo de Calibração)
STATE_CALIBRATE_INIT = "STATE_CALIBRATE_INIT"
STATE_CALIBRATE_VALIDATE = "STATE_CALIBRATE_VALIDATE_POSITION"
STATE_CALIBRATE_SIMULATE_PICK = "STATE_CALIBRATE_SIMULATE_PICK"
STATE_CALIBRATE_APPROACH_POINT = "STATE_CALIBRATE_APPROACH_POINT"
STATE_CALIBRATE_SCREWING = "STATE_CALIBRATE_SCREWING"
STATE_CALIBRATE_RETURN = "STATE_CALIBRATE_RETURN"

-- Variáveis globais para comunicação entre threads
g_start_torque_monitoring = false
g_screwing_in_progress = false
g_torque_reached = false
g_drive_id = nil
g_target_coords = {x=0, y=0, z=0} -- Armazena as coordenadas recebidas

-- Parâmetros do Servo Drive da Parafusadeira (Drive D2)
SCREWDRIVER_SLAVE_ID = 3
SCREWDRIVER_IP = "127.0.0.1"
SCREWDRIVER_PORT = 60000
SCREWING_TIMEOUT = 5.0 -- Timeout para a operação de aparafusamento em segundos

-- Mapeamento de Registradores Modbus (Servidor AAS)
START_REGISTER = 4000
X_COORD_REGISTER = 4001
Y_COORD_REGISTER = 4002
Z_COORD_REGISTER = 4003

-- Mapeamento de Registradores Modbus (Drive D2 via RTU)
OP_MODE_ADDR = 424
TARGET_CURRENT_ADDR = 4004
ACTUAL_CURRENT_ADDR = 104
TARGET_VELOCITY_ADDR = 6

-- Parâmetros do Motor e do Processo de Aparafusamento
MOTOR_KT = 0.356
INITIAL_TARGET_CURRENT_REG_VAL = 800

---
-- @description Função auxiliar para conectar e configurar a parafusadeira.
-- @return Retorna o ID da conexão se sucesso, ou nil se falhar.
---
function connectAndConfigureScrewdriver()
     ToolAnalogMode(00)
     SetTool485(9600,"E",1)
    print("→ Sub-rotina: Conectando e configurando a parafusadeira...")
    err, drive_id = ModbusCreate(SCREWDRIVER_IP, SCREWDRIVER_PORT, SCREWDRIVER_SLAVE_ID, 1)
    if err ~= 0 then
        print("✗ ERRO: Falha ao conectar com o drive da parafusadeira. Código: " .. err)
        return nil
    end
    print("✓ Conexão com a parafusadeira estabelecida. ID: " .. drive_id)

    local op_mode_value_to_set = {3}
    SetHoldRegs(drive_id, OP_MODE_ADDR, 1, op_mode_value_to_set, "U32")
    Sleep(50)
    local read_back_data = GetHoldRegs(drive_id, OP_MODE_ADDR, 1, "U32")
    if not (read_back_data[1] == op_mode_value_to_set[1]) then
        print("✗ ERRO: Falha ao configurar o modo de torque.")
        ModbusClose(drive_id)
        return nil
    end

    print("✓ Modo de torque da parafusadeira configurado.")
    return drive_id
end

function readTargetCoordinates()
    print("Lendo coordenadas do servidor MODBUS")
    data_x = GetHoldRegs(id, X_COORD_REGISTER, 1, "U16")[1]
    data_y = GetHoldRegs(id, Y_COORD_REGISTER, 1, "U16")[1]
    data_z = GetHoldRegs(id, Z_COORD_REGISTER, 1, "U16")[1]

    --if data_x !=0 and data_y !=0 and data_z !=0 then
        g_target_coords.x = data_x*0.01
        g_target_coords.y = data_y*0.01
        g_target_coords.z = data_z*0.01
        --print(string.format("Coordenadas recebidas: X=%.2f, Y=%.2f, Z=%.2f", g_target_coords.x, g_target_coords.y, g_target_coords.z))
        --return true
    --end
    
    --print("ERRO: Falha ao ler as coordenadas dos registradores Modbus.")
    --return false
end

function resetStartState()
    print("Finalizando ciclo. Escrevendo 0 no registradores")
    SetHoldRegs(id, START_REGISTER, 1, {0}, "U16")
    SetHoldRegs(id, X_COORD_REGISTER, 1, {0}, "U16")
    SetHoldRegs(id, Y_COORD_REGISTER, 1, {0}, "U16")
    SetHoldRegs(id, Z_COORD_REGISTER, 1, {0}, "U16")
    Sleep(50) -- Garante que a escrita seja processada
end

function TurnScrewdriver(status, direction)
    if not g_drive_id then
        print("ERRO (TurnScrewdriver): Conexão com o drive (g_drive_id) não está ativa.")
        return
    end

    -- Define a direção padrão como 'reverse' se não for especificada
    local effective_direction = direction or "reverse"
    local current_value_to_set

    if status == "OFF" then
        print("Desligando parafusadeira...")
        current_value_to_set = {0}
    elseif status == "ON" then
        if effective_direction == "reverse" then
            print("Ligando parafusadeira (sentido anti-horário)...")
            current_value_to_set = {toUint32(-INITIAL_TARGET_CURRENT_REG_VAL)}
        elseif effective_direction == "forward" then
            print("Ligando parafusadeira (sentido horário)...")
            current_value_to_set = {toUint32(INITIAL_TARGET_CURRENT_REG_VAL)}
        else
            print("ERRO (TurnScrewdriver): Direção inválida. Use 'forward' ou 'reverse'.")
            return
        end
    else
        print("ERRO (TurnScrewdriver): Status inválido. Use 'ON' ou 'OFF'.")
        return
    end
    
    -- Envia o comando de corrente para o drive
    SetHoldRegs(g_drive_id, TARGET_CURRENT_ADDR, 1, current_value_to_set, "U32")
    Sleep(50) -- Pequena pausa para garantir o envio do comando
end

function SetScrewdriverRPM(rpm)
    if not g_drive_id then
        print("ERRO (SetScrewdriverRPM): Conexão com o drive (g_drive_id) não está ativa.")
        return
    end
    
    if type(rpm) ~= "number" or rpm < 0 then
        print("ERRO (SetScrewdriverRPM): RPM deve ser um número positivo.")
        return
    end

    -- Fator de conversão: 10000 counts/sec = 1 rev/sec
    -- 1 RPM = (10000 / 60) counts/sec
    local counts_per_second = rpm * (10000 / 60)

    print(string.format("Ajustando RPM da parafusadeira para %.0f (%.2f counts/s)...", rpm, counts_per_second))

    -- Envia o comando de velocidade para o drive como um float de 32 bits
    SetHoldRegs(g_drive_id, TARGET_VELOCITY_ADDR, 1, {counts_per_second}, "F32")
    Sleep(30) -- Pequena pausa para garantir o envio do comando
end

-- Função para converter um INT32 com sinal para um U32 para envio pelo Dobot
function toUint32(signed_int)
    -- Se o número for negativo, calcula o complemento de dois
    if signed_int < 0 then
        -- Adiciona 2^32 ao número negativo para obter o equivalente U32
        return signed_int + 4294967296 
    end
    return signed_int
end

print("Definições globais carregadas.")