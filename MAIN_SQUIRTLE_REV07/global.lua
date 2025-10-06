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
start = 0
BoardX = 0
BoardY = 0
BoardZ = 0
att = 0


-- Variáveis globais para comunicação entre threads
g_start_torque_monitoring = false
g_screwing_in_progress = false
g_torque_reached = false
g_drive_id = nil

-- Parâmetros do Servo Drive da Parafusadeira (Drive D2)
SCREWDRIVER_SLAVE_ID = 3
SCREWDRIVER_IP = "127.0.0.1"
SCREWDRIVER_PORT = 60000

-- Registradores Modbus do Drive D2
OP_MODE_ADDR = 424
TARGET_CURRENT_ADDR = 4004
ACTUAL_CURRENT_ADDR = 104

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

print("Definições globais carregadas.")

-- Função para converter um INT32 com sinal para um U32 para envio pelo Dobot
function toUint32(signed_int)
    -- Se o número for negativo, calcula o complemento de dois
    if signed_int < 0 then
        -- Adiciona 2^32 ao número negativo para obter o equivalente U32
        return signed_int + 4294967296 
    end
    return signed_int
end

