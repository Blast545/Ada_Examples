pragma Warnings (Off);
pragma Ada_95;
pragma Source_File_Name (ada_main, Spec_File_Name => "b__main.ads");
pragma Source_File_Name (ada_main, Body_File_Name => "b__main.adb");
pragma Suppress (Overflow_Check);

with System.Restrictions;

package body ada_main is

   E093 : Short_Integer; pragma Import (Ada, E093, "system__soft_links_E");
   E091 : Short_Integer; pragma Import (Ada, E091, "system__exception_table_E");
   E057 : Short_Integer; pragma Import (Ada, E057, "ada__tags_E");
   E079 : Short_Integer; pragma Import (Ada, E079, "system__bb__timing_events_E");
   E127 : Short_Integer; pragma Import (Ada, E127, "ada__streams_E");
   E135 : Short_Integer; pragma Import (Ada, E135, "system__finalization_root_E");
   E133 : Short_Integer; pragma Import (Ada, E133, "ada__finalization_E");
   E137 : Short_Integer; pragma Import (Ada, E137, "system__storage_pools_E");
   E130 : Short_Integer; pragma Import (Ada, E130, "system__finalization_masters_E");
   E119 : Short_Integer; pragma Import (Ada, E119, "ada__real_time_E");
   E139 : Short_Integer; pragma Import (Ada, E139, "system__pool_global_E");
   E192 : Short_Integer; pragma Import (Ada, E192, "system__tasking__protected_objects_E");
   E198 : Short_Integer; pragma Import (Ada, E198, "system__tasking__protected_objects__multiprocessors_E");
   E211 : Short_Integer; pragma Import (Ada, E211, "system__tasking__restricted__stages_E");
   E230 : Short_Integer; pragma Import (Ada, E230, "cortex_m__cache_E");
   E216 : Short_Integer; pragma Import (Ada, E216, "hal__audio_E");
   E239 : Short_Integer; pragma Import (Ada, E239, "hal__block_drivers_E");
   E180 : Short_Integer; pragma Import (Ada, E180, "hal__gpio_E");
   E188 : Short_Integer; pragma Import (Ada, E188, "hal__i2c_E");
   E220 : Short_Integer; pragma Import (Ada, E220, "hal__real_time_clock_E");
   E234 : Short_Integer; pragma Import (Ada, E234, "hal__sdmmc_E");
   E125 : Short_Integer; pragma Import (Ada, E125, "hal__spi_E");
   E148 : Short_Integer; pragma Import (Ada, E148, "hal__time_E");
   E253 : Short_Integer; pragma Import (Ada, E253, "cs43l22_E");
   E250 : Short_Integer; pragma Import (Ada, E250, "hal__uart_E");
   E143 : Short_Integer; pragma Import (Ada, E143, "lis3dsh_E");
   E255 : Short_Integer; pragma Import (Ada, E255, "lis3dsh__spi_E");
   E147 : Short_Integer; pragma Import (Ada, E147, "ravenscar_time_E");
   E232 : Short_Integer; pragma Import (Ada, E232, "sdmmc_init_E");
   E157 : Short_Integer; pragma Import (Ada, E157, "stm32__adc_E");
   E163 : Short_Integer; pragma Import (Ada, E163, "stm32__dac_E");
   E204 : Short_Integer; pragma Import (Ada, E204, "stm32__dma__interrupts_E");
   E176 : Short_Integer; pragma Import (Ada, E176, "stm32__exti_E");
   E222 : Short_Integer; pragma Import (Ada, E222, "stm32__power_control_E");
   E219 : Short_Integer; pragma Import (Ada, E219, "stm32__rtc_E");
   E242 : Short_Integer; pragma Import (Ada, E242, "stm32__spi_E");
   E244 : Short_Integer; pragma Import (Ada, E244, "stm32__spi__dma_E");
   E184 : Short_Integer; pragma Import (Ada, E184, "stm32__i2c_E");
   E248 : Short_Integer; pragma Import (Ada, E248, "stm32__usarts_E");
   E238 : Short_Integer; pragma Import (Ada, E238, "stm32__sdmmc_interrupt_E");
   E215 : Short_Integer; pragma Import (Ada, E215, "stm32__i2s_E");
   E190 : Short_Integer; pragma Import (Ada, E190, "stm32__i2c__dma_E");
   E169 : Short_Integer; pragma Import (Ada, E169, "stm32__gpio_E");
   E227 : Short_Integer; pragma Import (Ada, E227, "stm32__sdmmc_E");
   E174 : Short_Integer; pragma Import (Ada, E174, "stm32__syscfg_E");
   E152 : Short_Integer; pragma Import (Ada, E152, "stm32__device_E");
   E150 : Short_Integer; pragma Import (Ada, E150, "stm32__setup_E");
   E124 : Short_Integer; pragma Import (Ada, E124, "stm32__board_E");
   E117 : Short_Integer; pragma Import (Ada, E117, "last_chance_handler_E");

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
         Count => (0, 0, 0, 1, 0, 0, 0, 0, 3, 0),
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
      E091 := E091 + 1;
      Ada.Tags'Elab_Body;
      E057 := E057 + 1;
      System.Bb.Timing_Events'Elab_Spec;
      E093 := E093 + 1;
      E079 := E079 + 1;
      Ada.Streams'Elab_Spec;
      E127 := E127 + 1;
      System.Finalization_Root'Elab_Spec;
      E135 := E135 + 1;
      Ada.Finalization'Elab_Spec;
      E133 := E133 + 1;
      System.Storage_Pools'Elab_Spec;
      E137 := E137 + 1;
      System.Finalization_Masters'Elab_Spec;
      System.Finalization_Masters'Elab_Body;
      E130 := E130 + 1;
      Ada.Real_Time'Elab_Body;
      E119 := E119 + 1;
      System.Pool_Global'Elab_Spec;
      E139 := E139 + 1;
      System.Tasking.Protected_Objects'Elab_Body;
      E192 := E192 + 1;
      System.Tasking.Protected_Objects.Multiprocessors'Elab_Body;
      E198 := E198 + 1;
      System.Tasking.Restricted.Stages'Elab_Body;
      E211 := E211 + 1;
      E230 := E230 + 1;
      HAL.AUDIO'ELAB_SPEC;
      E216 := E216 + 1;
      HAL.BLOCK_DRIVERS'ELAB_SPEC;
      E239 := E239 + 1;
      HAL.GPIO'ELAB_SPEC;
      E180 := E180 + 1;
      HAL.I2C'ELAB_SPEC;
      E188 := E188 + 1;
      HAL.REAL_TIME_CLOCK'ELAB_SPEC;
      E220 := E220 + 1;
      HAL.SDMMC'ELAB_SPEC;
      E234 := E234 + 1;
      HAL.SPI'ELAB_SPEC;
      E125 := E125 + 1;
      HAL.TIME'ELAB_SPEC;
      E148 := E148 + 1;
      CS43L22'ELAB_SPEC;
      CS43L22'ELAB_BODY;
      E253 := E253 + 1;
      HAL.UART'ELAB_SPEC;
      E250 := E250 + 1;
      LIS3DSH'ELAB_SPEC;
      LIS3DSH'ELAB_BODY;
      E143 := E143 + 1;
      LIS3DSH.SPI'ELAB_SPEC;
      LIS3DSH.SPI'ELAB_BODY;
      E255 := E255 + 1;
      Ravenscar_Time'Elab_Spec;
      Ravenscar_Time'Elab_Body;
      E147 := E147 + 1;
      E232 := E232 + 1;
      STM32.ADC'ELAB_SPEC;
      E157 := E157 + 1;
      E163 := E163 + 1;
      E204 := E204 + 1;
      E176 := E176 + 1;
      E222 := E222 + 1;
      STM32.RTC'ELAB_SPEC;
      STM32.RTC'ELAB_BODY;
      E219 := E219 + 1;
      STM32.SPI'ELAB_SPEC;
      STM32.SPI'ELAB_BODY;
      E242 := E242 + 1;
      STM32.SPI.DMA'ELAB_SPEC;
      STM32.SPI.DMA'ELAB_BODY;
      E244 := E244 + 1;
      STM32.I2C'ELAB_SPEC;
      STM32.USARTS'ELAB_SPEC;
      STM32.I2S'ELAB_SPEC;
      STM32.I2C.DMA'ELAB_SPEC;
      STM32.GPIO'ELAB_SPEC;
      STM32.SDMMC'ELAB_SPEC;
      E238 := E238 + 1;
      STM32.GPIO'ELAB_BODY;
      E169 := E169 + 1;
      STM32.DEVICE'ELAB_SPEC;
      E152 := E152 + 1;
      STM32.SDMMC'ELAB_BODY;
      E227 := E227 + 1;
      STM32.I2S'ELAB_BODY;
      E215 := E215 + 1;
      STM32.I2C.DMA'ELAB_BODY;
      E190 := E190 + 1;
      STM32.I2C'ELAB_BODY;
      E184 := E184 + 1;
      E174 := E174 + 1;
      STM32.USARTS'ELAB_BODY;
      E248 := E248 + 1;
      E150 := E150 + 1;
      STM32.BOARD'ELAB_SPEC;
      E124 := E124 + 1;
      E117 := E117 + 1;
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
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\adc_to_dac_audio\obj\last_chance_handler.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\adc_to_dac_audio\obj\main.o
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\adc_to_dac_audio\obj\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\adc_to_dac_audio\obj\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\shared\common\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\boards\stm32f407_discovery\obj\full_lib_Debug\
   --   -LC:\gnat\2018-arm-elf\arm-eabi\lib\gnat\ravenscar-full-stm32f4\adalib\
   --   -static
   --   -lgnarl
   --   -lgnat
--  END Object file/option list   

end ada_main;
