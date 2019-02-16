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

--  This is a demo of the features available on the STM32F4-DISCOVERY board.
--
--  Press the blue button to change the note of the sound played in the
--  headphone jack. Press the black button to reset.

--with HAL;                  use HAL;
--with HAL.Audio;            use HAL.Audio;
--with Simple_Synthesizer;
--with System;               use System;
--with Interfaces;           use Interfaces;
--with STM32.User_Button;
--with STM32.Device;         use STM32.Device;
--with STM32.Board;     use STM32.Board;
--with Audio_Stream;         use Audio_Stream;
--with Ada.Synchronous_Task_Control;  use Ada.Synchronous_Task_Control;
--with Ada.Real_Time;   use Ada.Real_Time;

with STM32.Board;          use STM32.Board;
with Last_Chance_Handler;  pragma Unreferenced (Last_Chance_Handler);

with STM32.Device;         use STM32.Device;
with STM32.GPIO;           use STM32.GPIO;
with System;               use System;
with STM32.Timers;         use STM32.Timers;
--with STM32.DMA; use STM32.DMA;
with HAL.Audio;            use HAL.Audio;
with STM32.ADC;            use STM32.ADC;
with ADC_Interrupt_Handling; use ADC_Interrupt_Handling;
with STM32.SPI;            use STM32.SPI;
with HAL.SPI;
with Ada.Synchronous_Task_Control;  use Ada.Synchronous_Task_Control;

with STM32F4_Timer_Interrupts;  pragma Unreferenced (STM32F4_Timer_Interrupts);
with HAL.Time;
with Ravenscar_Time;
with Interfaces;           use Interfaces;
with HAL;                  use HAL;
with ILI9341_Extended;     use ILI9341_Extended;
with Audio_Stream;         use Audio_Stream;

-- Project files used

procedure Main is

   HAL_Time  : constant HAL.Time.Any_Delays := Ravenscar_Time.Delays;

   -- ADC Variables
   MIC_GPIO  : GPIO_Point renames PB0;
   MIC_CONF  : constant GPIO_Port_Configuration := (Mode => Mode_Analog, Resistors => Floating);
   ADC_USED : Analog_To_Digital_Converter renames ADC_2;
   All_Regular_Conversions : constant Regular_Channel_Conversions :=
     (1 => (Channel => VMic.Channel, Sample_Time => Sample_15_Cycles));
   --ADC_Mic_Value : UInt16 := 0;

   -- Audio buffer variables
   Audio_Data_0 : Audio_Buffer (1 .. 64) := (others => 0);
   Audio_Data_1 : Audio_Buffer (1 .. 64) := (others => 0);
   Buffer_Sampled : samples;
   Processed_Data : Audio_Buffer(1 .. 32);
   Stereo_Data : Audio_Buffer(1 .. 64);
   Mid_Value : Float := 0.0;

   Screen : ILI9341_Device(Port => SPI_1'Access,
                           Chip_Select => PA1'Access,
                           WRX => PA2'Access,
                           Reset => PA3'Access,
                           Time => HAL_Time);
   SPI1_SCK     : GPIO_Point renames PA5;
   SPI1_MISO    : GPIO_Point renames PA6;
   SPI1_MOSI    : GPIO_Point renames PA7;

   procedure Initialize_ADC;
   procedure Initialize_Timer;
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
   procedure Initialize_Timer is
   begin
      Enable_Clock (Timer_7);
      Reset (Timer_7);
      -- Period defines the number of counts, prescaler divides the clock
      -- APB1 := 84 Mhz, Desired freq := APB1/Desired_sample rate
      -- Sample rate := 48khz, Period := 1750 @ Prescaler := 0
      Configure (Timer_7, Prescaler => 0, Period => 1750);
      Enable_Interrupt (Timer_7, Timer_Update_Interrupt);
      --Enable (Timer_7);
   end Initialize_Timer;


begin
   Initialize_LEDs;

   -- Initalize SPI clocks and GPIO
   Init_SPI_IO_Pins : declare
      Config : GPIO_Port_Configuration;
   begin
      Enable_Clock (GPIO_A);
      Enable_Clock (SPI_1);
      Enable_Clock (PA1 & PA2 & PA3 & SPI1_SCK & SPI1_MISO & SPI1_MOSI);


      Config := (Mode           => Mode_AF,
                 AF             => GPIO_AF_SPI1_5,
                 AF_Speed       => Speed_25MHz,
                 AF_Output_Type => Push_Pull,
                 Resistors      => Floating);

      Configure_IO (SPI1_SCK & SPI1_MISO & SPI1_MOSI, Config);
      Reset(SPI_1);
   end Init_SPI_IO_Pins;
   Init_SPI_Port : declare
         Config : SPI_Configuration;
      begin
         Config :=
           (Direction           => D2Lines_FullDuplex,
            Mode                => Master,
            Data_Size           => HAL.SPI.Data_Size_8b,
            Clock_Polarity      => Low,
            Clock_Phase         => P1Edge,
            Slave_Management    => Software_Managed,
            Baud_Rate_Prescaler => BRP_32,
            First_Bit           => MSB,
            CRC_Poly            => 7);

         Configure (SPI_1, Config);

      STM32.SPI.Enable (SPI_1);
      --All_LEDs_On;
   end Init_SPI_Port;
   Init_Chip_Select : declare
         Config : GPIO_Port_Configuration;
      begin
         Config := (Mode        => Mode_Out,
                    Speed       => Speed_25MHz,
                    Output_Type => Push_Pull,
                    Resistors   => Floating);

         Configure_IO (PA1 & PA2 & PA3, Config);
   end Init_Chip_Select;

   Initialize_Audio;
   --STM32.User_Button.Initialize;
   HAL_Time.Delay_Seconds(1);

   Initialize_ADC;
   -- Configure DAC DMA
   STM32.Board.Audio_DAC.Set_Volume (100);
   STM32.Board.Audio_DAC.Play;

   Initialize_Timer;

   -- Initialize ILI9341 screen
   Initialize(This => Screen, Mode => ILI9341_Extended.SPI_Mode);
   Screen.Set_Orientation(To   => Landscape_1);
   Screen.Set_Font(Font => Large_Font);
   HAL_Time.Delay_Microseconds(10);
   Fill(This  => Screen,
        Color => Black);

   -- Start the DMA function to send samples to DAC
   Audio_TX_DMA_Int.Start (Destination => STM32.Board.Audio_I2S.Data_Register_Address,
                           Source_0    => Audio_Data_0'Address,
                           Source_1    => Audio_Data_1'Address,
                           Data_Count  => Audio_Data_0'Length);
   -- Wait for the first dma transfer complete before starting processing the
   -- adc samples
   --Audio_TX_DMA_Int.Wait_For_Transfer_Complete;
   Enable (Timer_7);
   --Orange_LED.Toggle;

   loop

      -- Suspend until a new set of ADC samples is ready
      Suspend_Until_True(Completed_Set_Samples);

      -- Save a copy of the array of adc values
      Buffer_Sampled := Sampled_Data;

      --- Create an effect for your data
      -- Input: sampled data, output: effect data
      for Index in Buffer_Sampled'Range loop
         -- Example: adding effect to each sample
         Mid_Value := (Float(Buffer_Sampled(Index)) -2048.0) * 10.0;
         Processed_Data(Index) := Integer_16(Mid_Value);
      end loop;

      -- Converte the MONO signal to Stereo
      for Index in Processed_Data'Range loop
         Stereo_Data ((Index * 2) -1 ) := Processed_Data(Index);
         Stereo_Data ((Index * 2) ) := Processed_Data(Index);
      end loop;

      -- Help sync the dma transfers
      --Audio_TX_DMA_Int.Wait_For_Transfer_Complete;

      -- Send the new set of data to the DAC
      if Audio_TX_DMA_Int.Not_In_Transfer = Audio_Data_0'Address then
         Audio_Data_0 := Stereo_Data;
      else
         Audio_Data_1 := Stereo_Data;
      end if;
   end loop;

end Main;
