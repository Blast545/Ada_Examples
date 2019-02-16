pragma Warnings (Off);
pragma Ada_95;
pragma Source_File_Name (ada_main, Spec_File_Name => "b__main.ads");
pragma Source_File_Name (ada_main, Body_File_Name => "b__main.adb");
pragma Suppress (Overflow_Check);

with System.Restrictions;

package body ada_main is

   E005 : Short_Integer; pragma Import (Ada, E005, "ada__tags_E");
   E075 : Short_Integer; pragma Import (Ada, E075, "system__soft_links_E");
   E073 : Short_Integer; pragma Import (Ada, E073, "system__exception_table_E");
   E061 : Short_Integer; pragma Import (Ada, E061, "system__bb__timing_events_E");
   E177 : Short_Integer; pragma Import (Ada, E177, "ada__streams_E");
   E186 : Short_Integer; pragma Import (Ada, E186, "system__finalization_root_E");
   E184 : Short_Integer; pragma Import (Ada, E184, "ada__finalization_E");
   E188 : Short_Integer; pragma Import (Ada, E188, "system__storage_pools_E");
   E181 : Short_Integer; pragma Import (Ada, E181, "system__finalization_masters_E");
   E153 : Short_Integer; pragma Import (Ada, E153, "ada__real_time_E");
   E190 : Short_Integer; pragma Import (Ada, E190, "system__pool_global_E");
   E118 : Short_Integer; pragma Import (Ada, E118, "system__tasking__protected_objects_E");
   E126 : Short_Integer; pragma Import (Ada, E126, "system__tasking__protected_objects__multiprocessors_E");
   E135 : Short_Integer; pragma Import (Ada, E135, "system__tasking__restricted__stages_E");
   E221 : Short_Integer; pragma Import (Ada, E221, "cortex_m__cache_E");
   E207 : Short_Integer; pragma Import (Ada, E207, "hal__audio_E");
   E230 : Short_Integer; pragma Import (Ada, E230, "hal__block_drivers_E");
   E179 : Short_Integer; pragma Import (Ada, E179, "hal__gpio_E");
   E200 : Short_Integer; pragma Import (Ada, E200, "hal__i2c_E");
   E211 : Short_Integer; pragma Import (Ada, E211, "hal__real_time_clock_E");
   E225 : Short_Integer; pragma Import (Ada, E225, "hal__sdmmc_E");
   E234 : Short_Integer; pragma Import (Ada, E234, "hal__spi_E");
   E251 : Short_Integer; pragma Import (Ada, E251, "hal__time_E");
   E255 : Short_Integer; pragma Import (Ada, E255, "cs43l22_E");
   E242 : Short_Integer; pragma Import (Ada, E242, "hal__uart_E");
   E248 : Short_Integer; pragma Import (Ada, E248, "lis3dsh_E");
   E257 : Short_Integer; pragma Import (Ada, E257, "lis3dsh__spi_E");
   E250 : Short_Integer; pragma Import (Ada, E250, "ravenscar_time_E");
   E223 : Short_Integer; pragma Import (Ada, E223, "sdmmc_init_E");
   E244 : Short_Integer; pragma Import (Ada, E244, "simple_synthesizer_E");
   E149 : Short_Integer; pragma Import (Ada, E149, "stm32__adc_E");
   E159 : Short_Integer; pragma Import (Ada, E159, "stm32__dac_E");
   E204 : Short_Integer; pragma Import (Ada, E204, "stm32__dma__interrupts_E");
   E172 : Short_Integer; pragma Import (Ada, E172, "stm32__exti_E");
   E213 : Short_Integer; pragma Import (Ada, E213, "stm32__power_control_E");
   E210 : Short_Integer; pragma Import (Ada, E210, "stm32__rtc_E");
   E233 : Short_Integer; pragma Import (Ada, E233, "stm32__spi_E");
   E236 : Short_Integer; pragma Import (Ada, E236, "stm32__spi__dma_E");
   E196 : Short_Integer; pragma Import (Ada, E196, "stm32__i2c_E");
   E240 : Short_Integer; pragma Import (Ada, E240, "stm32__usarts_E");
   E229 : Short_Integer; pragma Import (Ada, E229, "stm32__sdmmc_interrupt_E");
   E206 : Short_Integer; pragma Import (Ada, E206, "stm32__i2s_E");
   E202 : Short_Integer; pragma Import (Ada, E202, "stm32__i2c__dma_E");
   E165 : Short_Integer; pragma Import (Ada, E165, "stm32__gpio_E");
   E218 : Short_Integer; pragma Import (Ada, E218, "stm32__sdmmc_E");
   E170 : Short_Integer; pragma Import (Ada, E170, "stm32__syscfg_E");
   E142 : Short_Integer; pragma Import (Ada, E142, "stm32__device_E");
   E116 : Short_Integer; pragma Import (Ada, E116, "audio_stream_E");
   E253 : Short_Integer; pragma Import (Ada, E253, "stm32__setup_E");
   E246 : Short_Integer; pragma Import (Ada, E246, "stm32__board_E");
   E259 : Short_Integer; pragma Import (Ada, E259, "stm32__user_button_E");

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
      E073 := E073 + 1;
      Ada.Tags'Elab_Body;
      E005 := E005 + 1;
      System.Bb.Timing_Events'Elab_Spec;
      E061 := E061 + 1;
      E075 := E075 + 1;
      Ada.Streams'Elab_Spec;
      E177 := E177 + 1;
      System.Finalization_Root'Elab_Spec;
      E186 := E186 + 1;
      Ada.Finalization'Elab_Spec;
      E184 := E184 + 1;
      System.Storage_Pools'Elab_Spec;
      E188 := E188 + 1;
      System.Finalization_Masters'Elab_Spec;
      System.Finalization_Masters'Elab_Body;
      E181 := E181 + 1;
      Ada.Real_Time'Elab_Body;
      E153 := E153 + 1;
      System.Pool_Global'Elab_Spec;
      E190 := E190 + 1;
      System.Tasking.Protected_Objects'Elab_Body;
      E118 := E118 + 1;
      System.Tasking.Protected_Objects.Multiprocessors'Elab_Body;
      E126 := E126 + 1;
      System.Tasking.Restricted.Stages'Elab_Body;
      E135 := E135 + 1;
      E221 := E221 + 1;
      HAL.AUDIO'ELAB_SPEC;
      E207 := E207 + 1;
      HAL.BLOCK_DRIVERS'ELAB_SPEC;
      E230 := E230 + 1;
      HAL.GPIO'ELAB_SPEC;
      E179 := E179 + 1;
      HAL.I2C'ELAB_SPEC;
      E200 := E200 + 1;
      HAL.REAL_TIME_CLOCK'ELAB_SPEC;
      E211 := E211 + 1;
      HAL.SDMMC'ELAB_SPEC;
      E225 := E225 + 1;
      HAL.SPI'ELAB_SPEC;
      E234 := E234 + 1;
      HAL.TIME'ELAB_SPEC;
      E251 := E251 + 1;
      CS43L22'ELAB_SPEC;
      CS43L22'ELAB_BODY;
      E255 := E255 + 1;
      HAL.UART'ELAB_SPEC;
      E242 := E242 + 1;
      LIS3DSH'ELAB_SPEC;
      LIS3DSH'ELAB_BODY;
      E248 := E248 + 1;
      LIS3DSH.SPI'ELAB_SPEC;
      LIS3DSH.SPI'ELAB_BODY;
      E257 := E257 + 1;
      Ravenscar_Time'Elab_Spec;
      Ravenscar_Time'Elab_Body;
      E250 := E250 + 1;
      E223 := E223 + 1;
      Simple_Synthesizer'Elab_Spec;
      Simple_Synthesizer'Elab_Body;
      E244 := E244 + 1;
      STM32.ADC'ELAB_SPEC;
      E149 := E149 + 1;
      E159 := E159 + 1;
      E204 := E204 + 1;
      E172 := E172 + 1;
      E213 := E213 + 1;
      STM32.RTC'ELAB_SPEC;
      STM32.RTC'ELAB_BODY;
      E210 := E210 + 1;
      STM32.SPI'ELAB_SPEC;
      STM32.SPI'ELAB_BODY;
      E233 := E233 + 1;
      STM32.SPI.DMA'ELAB_SPEC;
      STM32.SPI.DMA'ELAB_BODY;
      E236 := E236 + 1;
      STM32.I2C'ELAB_SPEC;
      STM32.USARTS'ELAB_SPEC;
      STM32.I2S'ELAB_SPEC;
      STM32.I2C.DMA'ELAB_SPEC;
      STM32.GPIO'ELAB_SPEC;
      STM32.SDMMC'ELAB_SPEC;
      E229 := E229 + 1;
      STM32.GPIO'ELAB_BODY;
      E165 := E165 + 1;
      STM32.DEVICE'ELAB_SPEC;
      E142 := E142 + 1;
      STM32.SDMMC'ELAB_BODY;
      E218 := E218 + 1;
      STM32.I2S'ELAB_BODY;
      E206 := E206 + 1;
      STM32.I2C.DMA'ELAB_BODY;
      E202 := E202 + 1;
      STM32.I2C'ELAB_BODY;
      E196 := E196 + 1;
      E170 := E170 + 1;
      STM32.USARTS'ELAB_BODY;
      E240 := E240 + 1;
      Audio_Stream'Elab_Spec;
      E116 := E116 + 1;
      E253 := E253 + 1;
      STM32.BOARD'ELAB_SPEC;
      E246 := E246 + 1;
      STM32.USER_BUTTON'ELAB_BODY;
      E259 := E259 + 1;
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
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\simple_audio\obj\audio_stream.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\simple_audio\obj\main.o
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\simple_audio\obj\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\simple_audio\obj\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\shared\common\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\boards\stm32f407_discovery\obj\full_lib_Debug\
   --   -LC:\gnat\2018-arm-elf\arm-eabi\lib\gnat\ravenscar-full-stm32f4\adalib\
   --   -static
   --   -lgnarl
   --   -lgnat
--  END Object file/option list   

end ada_main;
