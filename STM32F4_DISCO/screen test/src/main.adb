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

--with HAL;
with HAL.SPI; --use HAL.SPI;
with STM32.Board;          use STM32.Board;
--with HAL.Audio;            use HAL.Audio;
--with Simple_Synthesizer;
--with Audio_Stream;         use Audio_Stream;
--with System;               use System;
--with Interfaces;           use Interfaces;
with STM32.User_Button;
with STM32.Device;         use STM32.Device;
with STM32.SPI;            use STM32.SPI;
with STM32.GPIO;           use STM32.GPIO;

with ILI9341_Extended;     use ILI9341_Extended;
with HAL.Time;
with Ravenscar_Time;

procedure Main is

   HAL_Time  : constant HAL.Time.Any_Delays := Ravenscar_Time.Delays;

   Screen : ILI9341_Device(Port => SPI_1'Access,
                           Chip_Select => PA1'Access,
                           WRX => PA2'Access,
                           Reset => PA3'Access,
                           Time => HAL_Time);
   SPI1_SCK     : GPIO_Point renames PA5;
   SPI1_MISO    : GPIO_Point renames PA6;
   SPI1_MOSI    : GPIO_Point renames PA7;
   Mid_Value : Float := 0.0;

begin
   Initialize_LEDs;
   --Initialize_Audio;
   STM32.User_Button.Initialize;

   -- Initalize SPI clocks and GPIO
   Enable_Clock (GPIO_A);
   Enable_Clock (SPI_1);
   Enable_Clock (PA1 & PA2 & PA3 & SPI1_SCK & SPI1_MISO & SPI1_MOSI);

   Init_SPI_IO_Pins : declare
         Config : GPIO_Port_Configuration;
      begin
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

   -- Initialize ILI9341 screen
   Initialize(This => Screen, Mode => ILI9341_Extended.SPI_Mode);
   Screen.Set_Orientation(To   => Landscape_1);
   Screen.Set_Font(Font => Large_Font);

   --Enable_Display(This => Screen);

   HAL_Time.Delay_Microseconds(10);

   -- Some dummy code used to test the screen
   loop
      --  Update screen with a dummy line

      --Draw_Ver_Line(This             => Screen,
      --              X                => 0,
      --              Y                => 0,
      --              Height_In_Pixels => 20,
      --              Color            => Light_Blue);

      -- delay
      --HAL_Time.Delay_Microseconds(10);

      ---All_LEDs_On;

      --  Update screen with a dummy line
      Draw_Filled_Rectangle(This             => Screen,
                            X                => 100,
                            Y                => 100,
                            Width_In_Pixels  => 100,
                            Height_In_Pixels => 100,
                            Color            => White);
      Mid_Value := 54.56;
      Screen.Print_String(X                => 100,
                          Y                => 100,
                          Text             => Mid_Value'Img,
                          Foreground_Color => Red,
                          Background_Color => White);
      HAL_Time.Delay_Seconds(1);
--      Set_Pixel(This  => Screen,
--                X     => 20,
--                Y     => 20,
--                Color => Magenta);
      Fill(This  => Screen,
           Color => Magenta);
      HAL_Time.Delay_Seconds(1);

      Print_String(This             => Screen,
                   X                => 50,
                   Y                => 50,
                   Text             => "HELLO!",
                   Foreground_Color => Red,
                   Background_Color => White);

      HAL_Time.Delay_Seconds(1);

      -- delay
      Fill(This  => Screen,
           Color => Yellow);
      HAL_Time.Delay_Seconds(1);
      ---All_LEDs_Off;

   end loop;





end Main;
