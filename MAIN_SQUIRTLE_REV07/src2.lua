
--[[
==================================================================
  ARQUIVO: src1.lua 
  DESCRIÇÃO: Thread paralela de monitoramento 
==================================================================
--]]

print("[Thread Monitoramento]: Iniciada e aguardando...")

while true do
    if g_start_torque_monitoring then
        print("[Thread Monitoramento]: Sinal recebido. Iniciando monitoramento.")
        
        if g_drive_id == nil then
            print("[Thread Monitoramento]: ERRO FATAL - ID do drive não foi definido.")
            g_start_torque_monitoring = false
            goto continue_loop
        end

        while g_screwing_in_progress do
            local current_data = GetInRegs(g_drive_id, ACTUAL_CURRENT_ADDR, 1, "F32")
            if current_data and #current_data > 0 then
                local actual_torque_nm = current_data[1] -- * MOTOR_KT
                last_torque = actual_torque
                if actual_torque_nm >= 6  then
                    print(string.format("[Thread Monitoramento]: TORQUE ATINGIDO (%.4f Nm)!", actual_torque_nm))
                    
                    -- AÇÃO 1: Parar o movimento físico do robô.
                    --StopMove()
                    
                    -- AÇÃO 2: Sinalizar para a thread principal.
                    g_torque_reached = true
                    g_screwing_in_progress = false

                                        
                    break
                end
                print(string.format("[Thread Monitoramento]: TORQUE Atual (%.4f Nm)!", actual_torque_nm))
            end
            Sleep(20)
        end
        
        g_start_torque_monitoring = false 
        --print(string.format("[Thread Monitoramento]: TORQUE ATINGIDO (%.4f Nm)!", actual_torque_nm))
        --print("[Thread Monitoramento]: Monitoramento finalizado.")
    end

    ::continue_loop::
    Sleep(100)
end
