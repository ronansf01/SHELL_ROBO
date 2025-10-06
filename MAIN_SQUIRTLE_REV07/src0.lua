--[[ REV07
==================================================================
  ARQUIVO: src0.lua
  DESCRIÇÃO: Programa principal com a máquina de estados.
             Controla o movimento do robô e sinaliza para a
             thread de monitoramento quando iniciar a verificação de torque.
==================================================================
--]]

-- Configurações iniciais
SpeedFactor(80)
Accel(100)
Speed(100)
AccelS(100)
SpeedS(100)

-- Estados possíveis
local STATE_INIT = "INIT"
local STATE_IDLE = "IDLE"
local STATE_VALIDATE = "VALIDATE_POSITION"
local STATE_MANAGE_SCREW_INVENTORY = "MANAGE_SCREW_INVENTORY"
local STATE_PICK_SCREW = "PICK_SCREW"
local STATE_APPROACH_POINT = "APPROACH_POINT"
local STATE_EXECUTE_SCREWING = "EXECUTE_SCREWING"
local STATE_RESET = "RESET_REGISTER"
local STATE_RETRACT = "RETRACT"
local STATE_CLOSE = "CLOSE_MODBUS"
local STATE_ERROR = "ERROR"

-- Variáveis de controle
state = STATE_INIT
lastStart = 0
local id -- ID da conexão Modbus com o controlador principal (Slave 1)

-- Variáveis de posição
BoardX, BoardY, BoardZ = 0, 0, 0
local tam_bit = 3.63
local sqrt_2 = 1.41421356 
local posAprox = nil
local posFinal = nil
local robot_ip = '192.168.0.192'

-- Loop principal da máquina de estados
while true do
    print("Estado atual da máquina: " .. state)

    if state == STATE_INIT then
        local resultCreate
        resultCreate, id = ModbusCreate(robot_ip, 502, 1)
        if resultCreate == 0 then
            print("✓ Conexão Modbus (Principal) estabelecida.")
            
            g_drive_id = connectAndConfigureScrewdriver()
            if g_drive_id == nil then
                print("✗ Falha ao inicializar o drive da parafusadeira. Encerrando.")
                return
            end
            
            SetHoldRegs(id, 4004, 1, {0}, "U16") -- Reseta flag de erro
            SetHoldRegs(id, 4005, 1, {0}, "U16") -- Reseta flag de erro
            SetHoldRegs(id, 4000, 1, {0}, "U16")
            SetHoldRegs(id, 4001, 1, {0}, "U16")
            SetHoldRegs(id, 4002, 1, {0}, "U16")
            SetHoldRegs(id, 4003, 1, {0}, "U16")
           -- SetHoldRegs(id, 4008, 1, {18}, "U16")
            MoveJ(P1)
            state = STATE_IDLE
        else
            print("✗ Falha ao criar conexão Modbus (Principal).")
            return
        end

    elseif state == STATE_IDLE then
        if lastStart == 1  and start == 1 then
            state = STATE_VALIDATE
        end
        lastStart = start

    elseif state == STATE_VALIDATE then
       
        safe_switch = DI(5)
        pos_switch = DI(1)
        local posValida = (BoardX >= (0 + tam_bit) and BoardX < (140 - tam_bit)) and (BoardY >= (0 + tam_bit) and BoardY < (141 - tam_bit)) and (BoardZ >= 0 and BoardZ < 200) and (BoardY >= BoardX - 130 + (tam_bit * sqrt_2))
        
        if not posValida then
            print("✗fora dos limites de segurança!")
            SetHoldRegs(id, 4004, 1, {1}, "U16") -- Define erro
            state = STATE_ERROR
        elseif safe_switch == ON and pos_switch == ON then
            SetHoldRegs(id, 4004, 1, {0}, "U16") -- Sem erro
            state = STATE_PICK_SCREW
        end
       
        
     elseif state == STATE_PICK_SCREW then
        -- Executa a rotina de coleta
        Go(P2, "User=3 Tool=1 CP=1 Speed=100 Accel=20")
        Sync()
        -- LIGA O MOTOR PARA GIRAR ENQUANTO PEGA O PARAFUSO
        print("  Ligando motor da parafusadeira para a coleta...")
        local pick_rotation_command = toUint32(-INITIAL_TARGET_CURRENT_REG_VAL)
        SetHoldRegs(g_drive_id, TARGET_CURRENT_ADDR, 1, {pick_rotation_command}, "U32")
        
        Go(P3, "User=3 Tool=1 CP=1 Speed=5 Accel=5")
        Sync()
        -- DESLIGA O MOTOR APÓS A COLETA
        print("  Desligando motor da parafusadeira...")
        SetHoldRegs(g_drive_id, TARGET_CURRENT_ADDR, 1, {0}, "U32")
        Sleep(100) 
        
        Go(P2, "User=3 Tool=1 CP=1 Speed=40 Accel=20")
        Sync()

        -- Decrementa e atualiza o contador de parafusos no registrador Modbus
        local new_count = current_count
        SetHoldRegs(id, 4008, 1, {new_count}, "U16")
        --local QTD_PARA =  GetHoldRegs(id, 4008, 1, "U16")[1]
        --if QTD_PARA == 0 then
          --SetHoldRegs(id, 4008, 1, {18}, "U16")
        --end
        
        --print("✓ Parafuso coletado. Restam: " .. new_count)
        state = STATE_APPROACH_POINT -- Transiciona para o próximo estado

    elseif state == STATE_APPROACH_POINT then
        print("→ Estado: Movendo para aproximação...")
        posAprox = RP(P4, {BoardX, BoardY, -BoardZ - 80})
        posAprox1 = RP(P4, {BoardX, BoardY, -BoardZ - 10})
        posFinal = RP(P4, {BoardX, BoardY, -BoardZ + 1})
        Move(posAprox, "User=2 Tool=1 CP=1 SpeedS=30 AccelS=30")
        Move(posAprox1, "User=2 Tool=1 CP=1 SpeedS=20 AccelS=30")
        Sync()
        state = STATE_EXECUTE_SCREWING
 
    elseif state == STATE_EXECUTE_SCREWING then
        print("→ Estado: Executando aparafusamento...")
        --local drive_id_temp = connectAndConfigureScrewdriver()
        
        if g_drive_id then
            --g_drive_id = drive_id_temp
            local reverse_command = toUint32(-INITIAL_TARGET_CURRENT_REG_VAL)
            SetHoldRegs(g_drive_id, TARGET_CURRENT_ADDR, 1, {reverse_command}, "U32")
            
            g_torque_reached = false
            g_screwing_in_progress = true
            g_start_torque_monitoring = true

            Move(posFinal, "User=2 Tool=1 CP=1 SpeedS=1 AccelS=1")
            Sync()
            Move(posAprox, "User=2 Tool=1 CP=1 SpeedS=10 AccelS=30")

            g_screwing_in_progress = false -- Garante que a outra thread pare o loop interno

            if g_torque_reached then
                print("✓ Aparafusamento confirmado pela thread principal.")
                
                print("→ Parando o motor e fechando a conexão com o drive...")
                -- A thread principal agora é responsável por parar o motor
                SetHoldRegs(g_drive_id, TARGET_CURRENT_ADDR, 1, {0}, "U32")
                Sleep(50)
                ModbusClose(g_drive_id)
                g_drive_id = nil
                state = STATE_RETRACT
            else
                print("✗ ERRO: Movimento terminou mas o torque não foi atingido.")
                SetHoldRegs(g_drive_id, TARGET_CURRENT_ADDR, 1, {0}, "U32") -- Garante que o motor pare
                SetHoldRegs(id, 4005, 1, {1}, "U16")
                Sleep(50)
                ModbusClose(g_drive_id)
                g_drive_id = nil
                state = STATE_ERROR
            end
        else
            print("✗ ERRO: Falha ao conectar/configurar parafusadeira.")
            state = STATE_ERROR
        end

    elseif state == STATE_RETRACT then
        print("→ Estado: Recuando...")
        Move(posAprox, "User=2 Tool=1 CP=1 SpeedS=50 AccelS=50")
        Go(P1)
        Sync()
        state = STATE_RESET
        
    elseif state == STATE_RESET then
         
        -- Reseta registradores
        SetHoldRegs(id, 4000, 1, {0}, "U16")
        SetHoldRegs(id, 4001, 1, {0}, "U16")
        SetHoldRegs(id, 4002, 1, {0}, "U16")
        SetHoldRegs(id, 4003, 1, {0}, "U16")
        start = 0
      
        state = STATE_CLOSE

    elseif state == STATE_CLOSE then
        
        
        if g_drive_id ~= nil then
            print("→ Encerrando conexão com a parafusadeira...")
            ModbusClose(g_drive_id)
            g_drive_id = nil
        end
        
        ModbusClose(id)
        state = STATE_INIT

    elseif state == STATE_ERROR then
        start = 0
        print("⚠️ Processo em estado de ERRO.")
        -- Lógica de reset de erro
        
        local erroPosicao = GetHoldRegs(id, 4004, 1, "U16")[1]
        local erroParaf = GetHoldRegs(id, 4005, 1, "U16")[1]
        state = STATE_RESET
        if erroPosicao == 0 and erroParaf == 0 then
  
              state = STATE_RESET
        end
    end

    Sleep(250)
end
