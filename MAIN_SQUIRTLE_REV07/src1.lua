--[[
==================================================================
  ARQUIVO: src1.lua
  DESCRIÇÃO: Define variáveis lidas via monitoramento do modbus tc/pip
==================================================================

--]]

while true do

   
   local regs = GetHoldRegs(id, 4010, 1, "U16")
   att = regs[1]
   

   if att == 1 then
     DOExecute(1,ON) 
   else
    DOExecute(1,OFF) 
   end
   Sleep(1000)
   
end
