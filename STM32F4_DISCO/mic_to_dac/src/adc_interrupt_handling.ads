------------------------------------------------------------------------------
--                                                                          --
--                 Copyright (C) 2015-2016, AdaCore                         --
--                                                                          --
--  Redistribution and use in source and binary forms, with or without      --
--  modification, are permitted provided that the following conditions are  --
--  met:                                                                    --
--     1. Redistributions of source code must retain the above copyright    --
--        notice, this list of conditions and the following disclaimer.     --
--     2. Redistributions in binary form must reproduce the above copyright --
--        notice, this list of conditions and the following disclaimer in   --
--        the documentation and/or other materials provided with the        --
--        distribution.                                                     --
--     3. Neither the name of the copyright holder nor the names of its     --
--        contributors may be used to endorse or promote products derived   --
--        from this software without specific prior written permission.     --
--                                                                          --
--   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    --
--   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      --
--   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  --
--   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   --
--   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, --
--   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       --
--   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  --
--   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  --
--   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    --
--   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  --
--   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   --
--                                                                          --
------------------------------------------------------------------------------

--  This file provides an interrupt handler for ADC interrupts.

with Ada.Interrupts.Names;
with Ada.Synchronous_Task_Control;  use Ada.Synchronous_Task_Control;
with STM32.ADC;   use STM32.ADC;
with STM32.Device; use STM32.Device;
with HAL;                  use HAL;

package ADC_Interrupt_Handling is

   --Regular_Channel_EOC : Suspension_Object;
   Completed_Set_Samples : Suspension_Object;

   -- ADC 2, IN8, mapped to port PB0
   VMic_Channel : constant Analog_Input_Channel := 8;
   VMic : constant ADC_Point := (ADC_2'Access, Channel => VMic_Channel);

   -- Variables used to handle receiving the samples
   type samples is array (1 .. 32) of UInt16;
   Sampled_Data : samples;
   subtype Audio_Index is Natural range 1 .. 32;
   Current_Index_Samples : Audio_Index := Audio_Index'First;

   protected Handler is

      pragma Interrupt_Priority;

   private

      procedure IRQ_Handler with
        Attach_Handler => Ada.Interrupts.Names.ADC_Interrupt;

   end Handler;

end ADC_Interrupt_Handling;
