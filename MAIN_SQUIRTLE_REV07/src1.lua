--[[
==================================================================
  ARQUIVO: src1.lua
  DESCRIÇÃO: Define variáveis lidas via monitoramento do modbus tc/pip
==================================================================

--]]

while true do

   
   local regs = GetHoldRegs(id, 4000, 11, "U16")
   start = regs[1]
   BoardX = regs[2]*0.01
   BoardY = regs[3]*0.01
   BoardZ = regs[4]*0.01
   att = regs[11]
   

   if att == 1 then
     DOExecute(1,ON) 
   else
    DOExecute(1,OFF) 
   end
   Sleep(1000)
   
end
