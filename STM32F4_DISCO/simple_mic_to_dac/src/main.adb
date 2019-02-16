------------------------------------------------------------------------------
--                                                                          --
--                        Copyright (C) 2017, AdaCore                       --
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

--  This is a demo gets audio samples from an ADC microphone (MAX4466)
--  And outputs it using the STM32f407-Discovery DAC
--  Using ADC interrupts, and DMA transfers

with STM32.Board;          use STM32.Board;
with Last_Chance_Handler;  pragma Unreferenced (Last_Chance_Handler);

with STM32.Device;    use STM32.Device;
with STM32.GPIO;      use STM32.GPIO;
with System;               use System;
with STM32.DMA; use STM32.DMA;
with STM32.ADC;   use STM32.ADC;
with ADC_Interrupt_Handling; use ADC_Interrupt_Handling;
with Ada.Synchronous_Task_Control;  use Ada.Synchronous_Task_Control;
with Interfaces;           use Interfaces;
with HAL;          use HAL;

-- Project files used

procedure Main is

   MIC_GPIO  : GPIO_Point renames PB0;
   MIC_CONF  : constant GPIO_Port_Configuration := (Mode => Mode_Analog, Resistors => Floating);
   ADC_USED : Analog_To_Digital_Converter renames ADC_2;
   All_Regular_Conversions : constant Regular_Channel_Conversions :=
     (1 => (Channel => VMic.Channel, Sample_Time => Sample_15_Cycles));

   Counts : UInt32 := 0;
   Mid_Value : Float := 0.0;
   ADC_Processed : Integer_16:= 0 with Atomic;
   Value_from_ADC : UInt16 := 0;

  procedure Initialize_ADC;

  procedure Initialize_DMA;

  procedure Initialize_ADC is

   begin
      -- Initialize the GPIO port used for Analog input (PB0)
      Enable_Clock (GPIO_B);
      Configure_IO (MIC_GPIO, MIC_CONF);
      Enable_Clock(ADC_USED);

      -- Initialize ADC module for the used ADC
      Configure_Common_Properties
        (Mode           => Independent,
         Prescalar      => PCLK2_Div_2,
         DMA_Mode       => Disabled,
         Sampling_Delay => Sampling_Delay_5_Cycles);

      Configure_Unit
        (VMic.ADC.all,
         Resolution => ADC_Resolution_12_Bits,
         Alignment  => Right_Aligned);

      Configure_Regular_Conversions
        (VMic.ADC.all,
         Continuous  => False,
         Trigger     => Software_Triggered,
         Enable_EOC  => True,
         Conversions => All_Regular_Conversions);

      Enable_Interrupts (VMic.ADC.all, Regular_Channel_Conversion_Complete);
      Enable (VMic.ADC.all);
   end Initialize_ADC;

  procedure Initialize_DMA is
      Config : DMA_Stream_Configuration;
      Controller : DMA_Controller renames DMA_1;
      Stream : constant DMA_Stream_Selector := Stream_5;
      Selected_DMA_Channel : constant DMA_Channel_Selector := Channel_0;
      Transfer_Destination: constant Address := STM32.Board.Audio_I2S.Data_Register_Address;
      --Source_Destination: constant Address := ADC_Mic_Value'Address;
      Source_Destination: constant Address := ADC_Processed'Address;

   begin
      Enable_Clock (Controller);

      Reset (Controller, Stream);

      Config.Channel                      := Selected_DMA_Channel;
      Config.Direction                    := Memory_To_Peripheral;
      Config.Memory_Data_Format           := HalfWords;
      Config.Peripheral_Data_Format       := HalfWords;
      Config.Increment_Peripheral_Address := False;
      Config.Increment_Memory_Address     := False;
      Config.Operation_Mode               := Circular_Mode;
      Config.Priority                     := Priority_High;
      Config.FIFO_Enabled                 := False;
      Config.FIFO_Threshold               := FIFO_Threshold_Half_Full_Configuration;
      Config.Memory_Burst_Size            := Memory_Burst_Single;
      Config.Peripheral_Burst_Size        := Peripheral_Burst_Single;
      Configure (Controller, Stream, Config);

      Clear_All_Status (Controller, Stream);

      Configure_Data_Flow
        (Controller,
         Stream,
         Source      => Source_Destination,
         Destination => Transfer_Destination,
         Data_Count  => 1);  -- 1 halfword

      Enable (Controller, Stream);

   end Initialize_DMA;

begin
   Initialize_LEDs;
   -- Initialize I2S and DAC of the board
   Initialize_Audio;
   -- Configure ADC to receive samples from microphone sensor
   Initialize_ADC;
   -- Configure DAC DMA
   STM32.Board.Audio_DAC.Set_Volume (100);
   STM32.Board.Audio_DAC.Play;
   -- Use DMA to transfer automatically samples to DAC
   Initialize_DMA;

   loop
      -- Get an ADC sample, process it as an interrupt
      -- Wait until is ready to be used
      Start_Conversion (VMic.ADC.all);
      Suspend_Until_True (ADC_Interrupt_Handling.Regular_Channel_EOC);

      Value_from_ADC := Conversion_Value (VMic.ADC.all);
      Counts := UInt32 (Value_from_ADC);

      -- 2048 is the mid value of a 12bit value ADC sample
      -- this is substracted to get a signal going from -2048 to 2047
      Mid_Value := Float(Counts) - Float(2048.0);

      -- This mic requires amplification to produce a hearable signal
      -- Change this value according to the sounds to be fed to the mic
      Mid_Value := Mid_Value * 10.0;
      ADC_Processed := Integer_16(Mid_value);
   end loop;

end Main;
