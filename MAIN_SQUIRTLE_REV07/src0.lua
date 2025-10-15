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


-- Variáveis de controle
currentState = STATE_INIT
lastStart = 0
local id -- ID da conexão Modbus com o controlador principal (Slave 1)


local tam_bit = 3.63
local sqrt_2 = 1.41421356 
local posAprox = nil
local posFinal = nil
local robot_ip = '192.168.0.192'

-- Loop principal da máquina de estados
while true do
    print("Estado atual da máquina: " .. currentState)

    if currentState == STATE_INIT then
        local resultCreate
        resultCreate, id = ModbusCreate(robot_ip, 502, 1)
        if resultCreate == 0 then
            print("✓ Conexão Modbus (Principal) estabelecida.")
            
            g_drive_id = connectAndConfigureScrewdriver()
            if g_drive_id == nil then
                print("✗ Falha ao inicializar o drive da parafusadeira. Encerrando.")
                return
            end
            
            resetStartState()
            
            MoveJ(P1)
            currentState = STATE_IDLE
        else
            print("✗ Falha ao criar conexão Modbus (Principal).")
            return
        end

    elseif currentState == STATE_IDLE then
    
        readTargetCoordinates()
        --[[
        if readTargetCoordinates() then
            currentState = STATE_VALIDATE
        else
            currentState = STATE_ERROR
        end
        --]]
        
        local start_data = GetHoldRegs(id, START_REGISTER, 1, "U16")
        if start_data and #start_data > 0 then
            local start_command = start_data[1]
            
            if start_command == 1 then
                print("Comando de APARAFUSAMENTO (1) recebido. Transicionando para STATE_INIT.")
                currentState = STATE_VALIDATE
            elseif start_command == 2 then
                print("Comando de CALIBRAÇÃO (2) recebido. Transicionando para STATE_CALIBRATE_INIT.")
                currentState = STATE_CALIBRATE_INIT
            end
        end
        Sleep(200)

    elseif currentState == STATE_VALIDATE then
       
        safe_switch = DI(5)
        pos_switch = DI(1)
        local posValida = (g_target_coords.x >= (0 + tam_bit) and g_target_coords.x < (140 - tam_bit)) and (g_target_coords.y >= (0 + tam_bit) and g_target_coords.y < (141 - tam_bit)) and (g_target_coords.z >= 0 and g_target_coords.z < 200) and (g_target_coords.y >= g_target_coords.x - 130 + (tam_bit * sqrt_2))
        
        if not posValida then
            print("✗fora dos limites de segurança!")
            SetHoldRegs(id, 4004, 1, {1}, "U16") -- Define erro
            currentState = STATE_ERROR
        elseif safe_switch == ON and pos_switch == ON then
            SetHoldRegs(id, 4004, 1, {0}, "U16") -- Sem erro
            currentState = STATE_PICK_SCREW
        end
       
        
     elseif currentState == STATE_PICK_SCREW then
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
        currentState = STATE_APPROACH_POINT -- Transiciona para o próximo estado

    elseif currentState == STATE_APPROACH_POINT then
        print("→ Estado: Movendo para aproximação...")
        posAprox = RP(P4, {g_target_coords.x, g_target_coords.y, -g_target_coords.z - 80})
        posAprox1 = RP(P4, {g_target_coords.x, g_target_coords.y, -g_target_coords.z - 10})
        posFinal = RP(P4, {g_target_coords.x, g_target_coords.y, -g_target_coords.z + 1})
        Move(posAprox, "User=2 Tool=1 CP=1 SpeedS=30 AccelS=30")
        Move(posAprox1, "User=2 Tool=1 CP=1 SpeedS=20 AccelS=30")
        Sync()
        currentState = STATE_EXECUTE_SCREWING
 
    elseif currentState == STATE_EXECUTE_SCREWING then
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
                currentState = STATE_RETRACT
            else
                print("✗ ERRO: Movimento terminou mas o torque não foi atingido.")
                SetHoldRegs(g_drive_id, TARGET_CURRENT_ADDR, 1, {0}, "U32") -- Garante que o motor pare
                SetHoldRegs(id, 4005, 1, {1}, "U16")
                Sleep(50)
                ModbusClose(g_drive_id)
                g_drive_id = nil
                currentState = STATE_ERROR
            end
        else
            print("✗ ERRO: Falha ao conectar/configurar parafusadeira.")
            currentState = STATE_ERROR
        end

    elseif currentState == STATE_RETRACT then
        print("→ Estado: Recuando...")
        Move(posAprox, "User=2 Tool=1 CP=1 SpeedS=50 AccelS=50")
        Go(P1)
        Sync()
        currentState = STATE_RESET
        
    elseif currentState == STATE_RESET then
         
        -- Reseta registradores
        resetStartState()
        
        start = 0
      
        currentState = STATE_CLOSE

    elseif currentState == STATE_CLOSE then
        
        
        if g_drive_id ~= nil then
            print("→ Encerrando conexão com a parafusadeira...")
            ModbusClose(g_drive_id)
            g_drive_id = nil
        end
        
        ModbusClose(id)
        currentState = STATE_INIT

    elseif currentState == STATE_ERROR then
        start = 0
        print("⚠️ Processo em estado de ERRO.")
        -- Lógica de reset de erro
        
        local erroPosicao = GetHoldRegs(id, 4004, 1, "U16")[1]
        local erroParaf = GetHoldRegs(id, 4005, 1, "U16")[1]
        currentState = STATE_RESET
        if erroPosicao == 0 and erroParaf == 0 then 
              currentState = STATE_RESET
        end
        Sleep(200)
    end

----------------------------------------------------------------------
    -- INÍCIO: FLUXO DE CALIBRAÇÃO
    ----------------------------------------------------------------------
    if currentState == STATE_CALIBRATE_INIT then
        print("[CALIBRAÇÃO] Entrou em STATE_CALIBRATE_INIT")
        currentState = STATE_CALIBRATE_VALIDATE
        --[[
        if readTargetCoordinates() then
            currentState = STATE_CALIBRATE_SIMULATE_PICK
        else
            -- Em calibração, uma falha de leitura não vai para erro.
            print("[CALIBRAÇÃO] AVISO: Falha ao ler coordenadas. Retornando para IDLE.")
            resetStartState()
            currentState = STATE_IDLE
        end
        --]]
    elseif currentState == STATE_CALIBRATE_VALIDATE then
    
    posValida = (g_target_coords.x >= (0 + tam_bit) and g_target_coords.x < (140 - tam_bit)) and (g_target_coords.y >= (0 + tam_bit) and g_target_coords.y < (141 - tam_bit)) and (g_target_coords.z >= 0 and g_target_coords.z < 200) and (g_target_coords.y >= g_target_coords.x - 130 + (tam_bit * sqrt_2))
        
        if not posValida then
            print("✗fora dos limites de segurança!")
            SetHoldRegs(id, 4004, 1, {1}, "U16") -- Define erro
            currentState = STATE_ERROR
        else 
            SetHoldRegs(id, 4004, 1, {0}, "U16") -- Sem erro
            currentState = STATE_CALIBRATE_SIMULATE_PICK
        end
        
    elseif currentState == STATE_CALIBRATE_SIMULATE_PICK then
        print("[CALIBRAÇÃO] Entrou em STATE_CALIBRATE_SIMULATE_PICK")
        Go(P1, "User=3 Tool=1 CP=1 Speed=100 Accel=20")
        Sync()
        -- LIGA O MOTOR PARA GIRAR ENQUANTO PEGA O PARAFUSO
        print("  Ligando motor da parafusadeira para a coleta...")
        local pick_rotation_command = toUint32(-INITIAL_TARGET_CURRENT_REG_VAL)
        SetHoldRegs(g_drive_id, TARGET_CURRENT_ADDR, 1, {pick_rotation_command}, "U32")
        
        Move(P2, "User=3 Tool=1 CP=1 SpeedS=5 AccelS=5")
        Sync()
        -- DESLIGA O MOTOR APÓS A COLETA
        print("  Desligando motor da parafusadeira...")
        SetHoldRegs(g_drive_id, TARGET_CURRENT_ADDR, 1, {0}, "U32")
        Sleep(100) 
        
        Move(P1, "User=3 Tool=1 CP=1 SpeedS=10 AccelS=20")
        Sync()
        -- Bypass do estado de aproximação geral, indo direto para o ponto de aparafusamento
        currentState = STATE_CALIBRATE_APPROACH_POINT

    elseif currentState == STATE_CALIBRATE_APPROACH_POINT then
        print("[CALIBRAÇÃO] Entrou em STATE_CALIBRATE_APPROACH_POINT")
        print("→ Estado: Movendo para aproximação...")
        posAprox = RP(P4, {g_target_coords.x, g_target_coords.y, -g_target_coords.z - 80})
        posAprox1 = RP(P4, {g_target_coords.x, g_target_coords.y, -g_target_coords.z - 10})
        posFinal = RP(P4, {g_target_coords.x, g_target_coords.y, -g_target_coords.z + 1})
        Move(posAprox, "User=2 Tool=1 CP=1 SpeedS=30 AccelS=30")
        Move(posAprox1, "User=2 Tool=1 CP=1 SpeedS=20 AccelS=30")
        Sync()
        currentState = STATE_CALIBRATE_SCREWING
        
    elseif currentState == STATE_CALIBRATE_SCREWING then
        print("[CALIBRAÇÃO] Entrou em STATE_CALIBRATE_SCREWING")
        
        -- LIGA O MOTOR PARA GIRAR ENQUANTO PEGA O PARAFUSO
        print("  Ligando motor da parafusadeira para a coleta...")
        pick_rotation_command = toUint32(-INITIAL_TARGET_CURRENT_REG_VAL)
        SetHoldRegs(g_drive_id, TARGET_CURRENT_ADDR, 1, {pick_rotation_command}, "U32")
        
        Move(posFinal, "User=2 Tool=1 CP=1 SpeedS=1 AccelS=1")
        Sync()
        
        -- DESLIGA O MOTOR APÓS AO APARAFUSAMENTO
        print("  Desligando motor da parafusadeira...")
        SetHoldRegs(g_drive_id, TARGET_CURRENT_ADDR, 1, {0}, "U32")
        Sleep(200) 
        Move(posAprox, "User=2 Tool=1 CP=1 SpeedS=10 AccelS=30")
        Sync()
                   
            -- Independentemente do resultado, o ciclo continua
            currentState = STATE_CALIBRATE_RETURN
        
        
    elseif currentState == STATE_CALIBRATE_RETURN then
        print("[CALIBRAÇÃO] Entrou em STATE_CALIBRATE_RETURN")
        print("→ Estado: Recuando...")
        Move(posAprox, "User=2 Tool=1 CP=1 SpeedS=50 AccelS=50")
        Go(P1)
        Sync()
        print("Ciclo de calibração concluído.")
        resetStartState()
        currentState = STATE_IDLE
    end    
end
