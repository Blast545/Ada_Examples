pragma Warnings (Off);
pragma Ada_95;
pragma Source_File_Name (ada_main, Spec_File_Name => "b__main.ads");
pragma Source_File_Name (ada_main, Body_File_Name => "b__main.adb");
pragma Suppress (Overflow_Check);

with System.Restrictions;

package body ada_main is

   E009 : Short_Integer; pragma Import (Ada, E009, "system__soft_links_E");
   E045 : Short_Integer; pragma Import (Ada, E045, "system__exception_table_E");
   E088 : Short_Integer; pragma Import (Ada, E088, "ada__tags_E");
   E102 : Short_Integer; pragma Import (Ada, E102, "system__bb__timing_events_E");
   E173 : Short_Integer; pragma Import (Ada, E173, "ada__streams_E");
   E182 : Short_Integer; pragma Import (Ada, E182, "system__finalization_root_E");
   E180 : Short_Integer; pragma Import (Ada, E180, "ada__finalization_E");
   E184 : Short_Integer; pragma Import (Ada, E184, "system__storage_pools_E");
   E177 : Short_Integer; pragma Import (Ada, E177, "system__finalization_masters_E");
   E142 : Short_Integer; pragma Import (Ada, E142, "ada__real_time_E");
   E186 : Short_Integer; pragma Import (Ada, E186, "system__pool_global_E");
   E118 : Short_Integer; pragma Import (Ada, E118, "system__tasking__protected_objects_E");
   E122 : Short_Integer; pragma Import (Ada, E122, "system__tasking__protected_objects__multiprocessors_E");
   E127 : Short_Integer; pragma Import (Ada, E127, "system__tasking__restricted__stages_E");
   E266 : Short_Integer; pragma Import (Ada, E266, "bmp_fonts_E");
   E225 : Short_Integer; pragma Import (Ada, E225, "cortex_m__cache_E");
   E211 : Short_Integer; pragma Import (Ada, E211, "hal__audio_E");
   E234 : Short_Integer; pragma Import (Ada, E234, "hal__block_drivers_E");
   E175 : Short_Integer; pragma Import (Ada, E175, "hal__gpio_E");
   E196 : Short_Integer; pragma Import (Ada, E196, "hal__i2c_E");
   E215 : Short_Integer; pragma Import (Ada, E215, "hal__real_time_clock_E");
   E229 : Short_Integer; pragma Import (Ada, E229, "hal__sdmmc_E");
   E238 : Short_Integer; pragma Import (Ada, E238, "hal__spi_E");
   E255 : Short_Integer; pragma Import (Ada, E255, "hal__time_E");
   E259 : Short_Integer; pragma Import (Ada, E259, "cs43l22_E");
   E246 : Short_Integer; pragma Import (Ada, E246, "hal__uart_E");
   E263 : Short_Integer; pragma Import (Ada, E263, "ili9341_extended_E");
   E252 : Short_Integer; pragma Import (Ada, E252, "lis3dsh_E");
   E261 : Short_Integer; pragma Import (Ada, E261, "lis3dsh__spi_E");
   E254 : Short_Integer; pragma Import (Ada, E254, "ravenscar_time_E");
   E227 : Short_Integer; pragma Import (Ada, E227, "sdmmc_init_E");
   E138 : Short_Integer; pragma Import (Ada, E138, "stm32__adc_E");
   E155 : Short_Integer; pragma Import (Ada, E155, "stm32__dac_E");
   E208 : Short_Integer; pragma Import (Ada, E208, "stm32__dma__interrupts_E");
   E168 : Short_Integer; pragma Import (Ada, E168, "stm32__exti_E");
   E217 : Short_Integer; pragma Import (Ada, E217, "stm32__power_control_E");
   E214 : Short_Integer; pragma Import (Ada, E214, "stm32__rtc_E");
   E237 : Short_Integer; pragma Import (Ada, E237, "stm32__spi_E");
   E240 : Short_Integer; pragma Import (Ada, E240, "stm32__spi__dma_E");
   E192 : Short_Integer; pragma Import (Ada, E192, "stm32__i2c_E");
   E244 : Short_Integer; pragma Import (Ada, E244, "stm32__usarts_E");
   E233 : Short_Integer; pragma Import (Ada, E233, "stm32__sdmmc_interrupt_E");
   E210 : Short_Integer; pragma Import (Ada, E210, "stm32__i2s_E");
   E198 : Short_Integer; pragma Import (Ada, E198, "stm32__i2c__dma_E");
   E161 : Short_Integer; pragma Import (Ada, E161, "stm32__gpio_E");
   E222 : Short_Integer; pragma Import (Ada, E222, "stm32__sdmmc_E");
   E166 : Short_Integer; pragma Import (Ada, E166, "stm32__syscfg_E");
   E148 : Short_Integer; pragma Import (Ada, E148, "stm32__device_E");
   E129 : Short_Integer; pragma Import (Ada, E129, "adc_interrupt_handling_E");
   E257 : Short_Integer; pragma Import (Ada, E257, "stm32__setup_E");
   E250 : Short_Integer; pragma Import (Ada, E250, "stm32__board_E");
   E248 : Short_Integer; pragma Import (Ada, E248, "audio_stream_E");
   E268 : Short_Integer; pragma Import (Ada, E268, "last_chance_handler_E");
   E270 : Short_Integer; pragma Import (Ada, E270, "stm32f4_timer_interrupts_E");

   Sec_Default_Sized_Stacks : array (1 .. 1) of aliased System.Secondary_Stack.SS_Stack (System.Parameters.Runtime_Default_Sec_Stack_Size);

   Local_Priority_Specific_Dispatching : constant String := "";
   Local_Interrupt_States : constant String := "";

   Is_Elaborated : Boolean := False;

   procedure adafinal is
      procedure s_stalib_adafinal;
      pragma Import (C, s_stalib_adafinal, "system__standard_library__adafinal");

      procedure Runtime_Finalize;
      pragma Import (C, Runtime_Finalize, "__gnat_runtime_finalize");

   begin
      if not Is_Elaborated then
         return;
      end if;
      Is_Elaborated := False;
      Runtime_Finalize;
      s_stalib_adafinal;
   end adafinal;

   procedure adainit is
      Main_Priority : Integer;
      pragma Import (C, Main_Priority, "__gl_main_priority");
      Time_Slice_Value : Integer;
      pragma Import (C, Time_Slice_Value, "__gl_time_slice_val");
      WC_Encoding : Character;
      pragma Import (C, WC_Encoding, "__gl_wc_encoding");
      Locking_Policy : Character;
      pragma Import (C, Locking_Policy, "__gl_locking_policy");
      Queuing_Policy : Character;
      pragma Import (C, Queuing_Policy, "__gl_queuing_policy");
      Task_Dispatching_Policy : Character;
      pragma Import (C, Task_Dispatching_Policy, "__gl_task_dispatching_policy");
      Priority_Specific_Dispatching : System.Address;
      pragma Import (C, Priority_Specific_Dispatching, "__gl_priority_specific_dispatching");
      Num_Specific_Dispatching : Integer;
      pragma Import (C, Num_Specific_Dispatching, "__gl_num_specific_dispatching");
      Main_CPU : Integer;
      pragma Import (C, Main_CPU, "__gl_main_cpu");
      Interrupt_States : System.Address;
      pragma Import (C, Interrupt_States, "__gl_interrupt_states");
      Num_Interrupt_States : Integer;
      pragma Import (C, Num_Interrupt_States, "__gl_num_interrupt_states");
      Unreserve_All_Interrupts : Integer;
      pragma Import (C, Unreserve_All_Interrupts, "__gl_unreserve_all_interrupts");
      Detect_Blocking : Integer;
      pragma Import (C, Detect_Blocking, "__gl_detect_blocking");
      Default_Stack_Size : Integer;
      pragma Import (C, Default_Stack_Size, "__gl_default_stack_size");
      Default_Secondary_Stack_Size : System.Parameters.Size_Type;
      pragma Import (C, Default_Secondary_Stack_Size, "__gnat_default_ss_size");
      Leap_Seconds_Support : Integer;
      pragma Import (C, Leap_Seconds_Support, "__gl_leap_seconds_support");
      Bind_Env_Addr : System.Address;
      pragma Import (C, Bind_Env_Addr, "__gl_bind_env_addr");

      procedure Runtime_Initialize (Install_Handler : Integer);
      pragma Import (C, Runtime_Initialize, "__gnat_runtime_initialize");
      Binder_Sec_Stacks_Count : Natural;
      pragma Import (Ada, Binder_Sec_Stacks_Count, "__gnat_binder_ss_count");
      Default_Sized_SS_Pool : System.Address;
      pragma Import (Ada, Default_Sized_SS_Pool, "__gnat_default_ss_pool");

   begin
      if Is_Elaborated then
         return;
      end if;
      Is_Elaborated := True;
      Main_Priority := -1;
      Time_Slice_Value := 0;
      WC_Encoding := 'b';
      Locking_Policy := 'C';
      Queuing_Policy := ' ';
      Task_Dispatching_Policy := 'F';
      System.Restrictions.Run_Time_Restrictions :=
        (Set =>
          (False, True, True, False, False, False, False, True, 
           False, False, False, False, False, False, False, True, 
           True, False, False, False, False, False, True, False, 
           False, False, False, False, False, False, False, False, 
           True, True, False, False, True, True, False, False, 
           False, True, False, False, False, False, True, False, 
           True, True, False, False, False, False, True, True, 
           True, True, True, False, True, False, False, False, 
           False, False, False, False, False, False, False, False, 
           False, False, False, False, False, True, False, False, 
           False, False, False, False, False, False, True, True, 
           False, True, False, False),
         Value => (0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
         Violated =>
          (False, False, False, False, True, True, False, False, 
           False, False, False, True, True, True, True, False, 
           False, False, False, False, True, True, False, True, 
           True, False, True, True, False, True, False, False, 
           False, False, False, True, False, False, True, False, 
           False, False, True, True, False, False, False, True, 
           False, False, False, True, False, False, False, False, 
           False, False, False, False, False, True, True, True, 
           False, False, True, False, True, True, True, False, 
           True, True, False, False, True, True, True, False, 
           False, True, False, False, False, True, False, False, 
           False, False, True, False),
         Count => (0, 0, 0, 1, 0, 0, 0, 0, 5, 0),
         Unknown => (False, False, False, False, False, False, False, False, True, False));
      Priority_Specific_Dispatching :=
        Local_Priority_Specific_Dispatching'Address;
      Num_Specific_Dispatching := 0;
      Main_CPU := -1;
      Interrupt_States := Local_Interrupt_States'Address;
      Num_Interrupt_States := 0;
      Unreserve_All_Interrupts := 0;
      Detect_Blocking := 1;
      Default_Stack_Size := -1;
      Leap_Seconds_Support := 0;

      ada_main'Elab_Body;
      Default_Secondary_Stack_Size := System.Parameters.Runtime_Default_Sec_Stack_Size;
      Binder_Sec_Stacks_Count := 1;
      Default_Sized_SS_Pool := Sec_Default_Sized_Stacks'Address;

      Runtime_Initialize (1);

      System.Soft_Links'Elab_Spec;
      System.Exception_Table'Elab_Body;
      E045 := E045 + 1;
      Ada.Tags'Elab_Body;
      E088 := E088 + 1;
      System.Bb.Timing_Events'Elab_Spec;
      E009 := E009 + 1;
      E102 := E102 + 1;
      Ada.Streams'Elab_Spec;
      E173 := E173 + 1;
      System.Finalization_Root'Elab_Spec;
      E182 := E182 + 1;
      Ada.Finalization'Elab_Spec;
      E180 := E180 + 1;
      System.Storage_Pools'Elab_Spec;
      E184 := E184 + 1;
      System.Finalization_Masters'Elab_Spec;
      System.Finalization_Masters'Elab_Body;
      E177 := E177 + 1;
      Ada.Real_Time'Elab_Body;
      E142 := E142 + 1;
      System.Pool_Global'Elab_Spec;
      E186 := E186 + 1;
      System.Tasking.Protected_Objects'Elab_Body;
      E118 := E118 + 1;
      System.Tasking.Protected_Objects.Multiprocessors'Elab_Body;
      E122 := E122 + 1;
      System.Tasking.Restricted.Stages'Elab_Body;
      E127 := E127 + 1;
      E266 := E266 + 1;
      E225 := E225 + 1;
      HAL.AUDIO'ELAB_SPEC;
      E211 := E211 + 1;
      HAL.BLOCK_DRIVERS'ELAB_SPEC;
      E234 := E234 + 1;
      HAL.GPIO'ELAB_SPEC;
      E175 := E175 + 1;
      HAL.I2C'ELAB_SPEC;
      E196 := E196 + 1;
      HAL.REAL_TIME_CLOCK'ELAB_SPEC;
      E215 := E215 + 1;
      HAL.SDMMC'ELAB_SPEC;
      E229 := E229 + 1;
      HAL.SPI'ELAB_SPEC;
      E238 := E238 + 1;
      HAL.TIME'ELAB_SPEC;
      E255 := E255 + 1;
      CS43L22'ELAB_SPEC;
      CS43L22'ELAB_BODY;
      E259 := E259 + 1;
      HAL.UART'ELAB_SPEC;
      E246 := E246 + 1;
      Ili9341_Extended'Elab_Spec;
      Ili9341_Extended'Elab_Body;
      E263 := E263 + 1;
      LIS3DSH'ELAB_SPEC;
      LIS3DSH'ELAB_BODY;
      E252 := E252 + 1;
      LIS3DSH.SPI'ELAB_SPEC;
      LIS3DSH.SPI'ELAB_BODY;
      E261 := E261 + 1;
      Ravenscar_Time'Elab_Spec;
      Ravenscar_Time'Elab_Body;
      E254 := E254 + 1;
      E227 := E227 + 1;
      STM32.ADC'ELAB_SPEC;
      E138 := E138 + 1;
      E155 := E155 + 1;
      E208 := E208 + 1;
      E168 := E168 + 1;
      E217 := E217 + 1;
      STM32.RTC'ELAB_SPEC;
      STM32.RTC'ELAB_BODY;
      E214 := E214 + 1;
      STM32.SPI'ELAB_SPEC;
      STM32.SPI'ELAB_BODY;
      E237 := E237 + 1;
      STM32.SPI.DMA'ELAB_SPEC;
      STM32.SPI.DMA'ELAB_BODY;
      E240 := E240 + 1;
      STM32.I2C'ELAB_SPEC;
      STM32.USARTS'ELAB_SPEC;
      STM32.I2S'ELAB_SPEC;
      STM32.I2C.DMA'ELAB_SPEC;
      STM32.GPIO'ELAB_SPEC;
      STM32.SDMMC'ELAB_SPEC;
      E233 := E233 + 1;
      STM32.GPIO'ELAB_BODY;
      E161 := E161 + 1;
      STM32.DEVICE'ELAB_SPEC;
      E148 := E148 + 1;
      STM32.SDMMC'ELAB_BODY;
      E222 := E222 + 1;
      STM32.I2S'ELAB_BODY;
      E210 := E210 + 1;
      STM32.I2C.DMA'ELAB_BODY;
      E198 := E198 + 1;
      STM32.I2C'ELAB_BODY;
      E192 := E192 + 1;
      E166 := E166 + 1;
      STM32.USARTS'ELAB_BODY;
      E244 := E244 + 1;
      Adc_Interrupt_Handling'Elab_Spec;
      E129 := E129 + 1;
      E257 := E257 + 1;
      STM32.BOARD'ELAB_SPEC;
      E250 := E250 + 1;
      Audio_Stream'Elab_Spec;
      E248 := E248 + 1;
      E268 := E268 + 1;
      Stm32f4_Timer_Interrupts'Elab_Spec;
      E270 := E270 + 1;
   end adainit;

   procedure Ada_Main_Program;
   pragma Import (Ada, Ada_Main_Program, "_ada_main");

   procedure main is
      procedure Initialize (Addr : System.Address);
      pragma Import (C, Initialize, "__gnat_initialize");

      procedure Finalize;
      pragma Import (C, Finalize, "__gnat_finalize");
      SEH : aliased array (1 .. 2) of Integer;

      Ensure_Reference : aliased System.Address := Ada_Main_Program_Name'Address;
      pragma Volatile (Ensure_Reference);

   begin
      Initialize (SEH'Address);
      adainit;
      Ada_Main_Program;
      adafinal;
      Finalize;
   end;

--  BEGIN Object file/option list
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\mic_to_dac\obj\bmp_fonts.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\mic_to_dac\obj\ili9341_extended.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\mic_to_dac\obj\adc_interrupt_handling.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\mic_to_dac\obj\audio_stream.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\mic_to_dac\obj\last_chance_handler.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\mic_to_dac\obj\stm32f4_timer_interrupts.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\mic_to_dac\obj\main.o
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\mic_to_dac\obj\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\mic_to_dac\obj\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\shared\common\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\boards\stm32f407_discovery\obj\full_lib_Debug\
   --   -LC:\gnat\2018-arm-elf\arm-eabi\lib\gnat\ravenscar-full-stm32f4\adalib\
   --   -static
   --   -lgnarl
   --   -lgnat
--  END Object file/option list   

end ada_main;
